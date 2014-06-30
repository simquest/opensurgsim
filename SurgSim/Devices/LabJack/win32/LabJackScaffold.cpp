// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "SurgSim/Devices/LabJack/LabJackScaffold.h"

#include <algorithm>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <iostream>
#include <LabJackUD.h> // the high-level LabJack library.
#include <list>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/LabJack/LabJackDevice.h"
#include "SurgSim/Devices/LabJack/LabJackThread.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"

namespace SurgSim
{
namespace Device
{

namespace
{
/// The LabJackUD returns a handle of 0 when failing to open a new device.
static const LJ_HANDLE LABJACK_INVALID_HANDLE = 0;

/// Returns true if there are no LabJackUD errors.
/// \param code The error code.
/// \return True if no LabJackUD errors.
bool isOk(LJ_ERROR code)
{
	return (code == LJE_NOERROR);
}

/// Outputs a string containing the LabJackUD error code and text equivalent.
/// \param errorCode The error code used by LabJackUD.
/// \return A string containing the message.
std::string formatErrorMessage(LJ_ERROR code)
{
	char error[256]; // According to LabJackUD.h, the buffer must store at least 256 elements.
	ErrorToString(code, error);
	return std::string("LabJackUD returned error code: ") + std::to_string(code) + ", and string: " + error;
}

/// A struct containing the default settings that depend on the model of LabJack.
struct LabJackDefaults
{
	LabJackDefaults()
	{
		timerBase[LabJack::MODEL_U3] = LabJack::TIMERBASE_22;
		timerBase[LabJack::MODEL_U6] = LabJack::TIMERBASE_22;
		timerBase[LabJack::MODEL_UE9] = LabJack::TIMERBASE_1;
	}

	/// The default timer base rate.
	std::unordered_map<LabJack::Model, LabJack::TimerBase> timerBase;
};
};

class LabJackScaffold::Handle
{
public:
	/// Constructor that attempts to open a device.
	/// \param model The model of LabJack device to open (see strings in LabJackUD.h).
	/// \param connection How to connect to the device (e.g., USB) (see strings in LabJackUD.h).
	/// \param address Either the ID or serial number (if USB), or the IP address.
	Handle(LabJack::Model model, LabJack::Connection connection,
		const std::string& address) :
		m_address(address),
		m_model(model),
		m_connection(connection),
		m_deviceHandle(LABJACK_INVALID_HANDLE),
		m_scaffold(LabJackScaffold::getOrCreateSharedInstance())
	{
		create();
	}

	/// Destructor.
	~Handle()
	{
		SURGSIM_ASSERT(!isValid()) << "Expected destroy() to be called before Handle object destruction.";
	}

	/// \return Whether or not the wrapped handle is valid.
	bool isValid() const
	{
		return (m_deviceHandle != LABJACK_INVALID_HANDLE);
	}

	/// Helper function called by the constructor to open the LabJack device for communications.
	void create()
	{
		SURGSIM_ASSERT(!isValid()) <<
			"Expected LabJackScaffold::Handle::create() to be called on an uninitialized object.";

		int firstFound = 0;
		if (m_address.length() == 0)
		{
			firstFound = 1;  // If no address is specified, grab the first device found of this model and connection.
		}

		const LJ_ERROR error = OpenLabJack(m_model, m_connection, m_address.c_str(), firstFound, &m_deviceHandle);
		SURGSIM_LOG_IF(!isOk(error), m_scaffold->getLogger(), SEVERE) <<
			"Failed to initialize a device. Model: " << m_model << ". Connection: " << m_connection << ". Address: '" <<
			m_address << "'." << std::endl << formatErrorMessage(error);
	}

	bool destroy()
	{
		bool result = false;
		if (isValid())
		{
			// Reset the pin configuration.
			const LJ_ERROR error = ePut(m_deviceHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
			result = isOk(error);
			SURGSIM_LOG_IF(!result, m_scaffold->getLogger(), SEVERE) <<
				"Failed to reset a device's pin configuration. Model: " << m_model << ". Connection: " << m_connection <<
				". Address: '" << m_address << "'." << std::endl << formatErrorMessage(error);
			if (result)
			{
				m_deviceHandle = LABJACK_INVALID_HANDLE;
			}
		}
		return result;
	}

	/// \return The LabJackUD's handle wrapped by this Handle.
	LJ_HANDLE get() const
	{
		return m_deviceHandle;
	}

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	Handle(const Handle&) /*= delete*/;
	Handle& operator=(const Handle&) /*= delete*/;

	/// The HDAL device handle (or LABJACK_INVALID_HANDLE if not valid).
	LJ_HANDLE m_deviceHandle;
	/// The address used to open the device.  Can be the empty string if the first-found device was opened.
	std::string m_address;
	/// The device model.
	LabJack::Model m_model;
	/// The connection to the device.
	LabJack::Connection m_connection;
	/// The scaffold.
	std::shared_ptr<LabJackScaffold> m_scaffold;
};

/// The per-device data.
struct LabJackScaffold::DeviceData
{
public:
	/// Initialize the data, creating a thread.
	DeviceData(LabJackDevice* device, std::unique_ptr<Handle>&& handle) :
		deviceObject(device),
		deviceHandle(std::move(handle)),
		thread(),
		digitalInputChannels(device->getDigitalInputChannels()),
		digitalOutputChannels(device->getDigitalOutputChannels()),
		timerInputChannels(getTimerInputChannels(device->getTimers())),
		timerOutputChannels(getTimerOutputChannels(device->getTimers())),
		analogInputsDifferential(device->getAnalogInputsDifferential()),
		analogInputsSingleEnded(device->getAnalogInputsSingleEnded()),
		analogOutputChannels(device->getAnalogOutputChannels()),
		cachedOutputIndices(false)
	{
	}


	~DeviceData()
	{
	}

	/// The corresponding device object.
	LabJackDevice* const deviceObject;
	/// Processing thread.
	std::unique_ptr<LabJackThread> thread;
	/// Device handle to read from.
	std::unique_ptr<Handle> deviceHandle;
	/// The channels read for digital inputs.
	const std::unordered_set<int> digitalInputChannels;
	/// The channels set for digital outputs.
	const std::unordered_set<int> digitalOutputChannels;
	/// The timer channels that provide inputs.
	const std::unordered_set<int> timerInputChannels;
	/// The timer channels set for timer outputs (e.g., PWM outputs).
	const std::unordered_set<int> timerOutputChannels;
	/// The differential analog inputs.
	const std::unordered_map<int, LabJack::RangeAndOptionalNegativeChannel> analogInputsDifferential;
	/// The single-ended analog inputs.
	const std::unordered_map<int, LabJack::Range> analogInputsSingleEnded;
	/// The channels set for analog outputs.
	const std::unordered_set<int> analogOutputChannels;
	/// The DataGroup indices for the digital outputs.
	std::unordered_map<int, int> digitalOutputIndices;
	/// The DataGroup indices for the digital inputs.
	std::unordered_map<int, int> digitalInputIndices;
	/// The DataGroup indices for the timer outputs.
	std::unordered_map<int, int> timerOutputIndices;
	/// The DataGroup indices for the timer inputs.
	std::unordered_map<int, int> timerInputIndices;
	/// The DataGroup indices for the analog outputs.
	std::unordered_map<int, int> analogOutputIndices;
	/// The DataGroup indices for the differential analog inputs.
	std::unordered_map<int, int> analogInputDifferentialIndices;
	/// The DataGroup indices for the single-ended analog inputs.
	std::unordered_map<int, int> analogInputSingleEndedIndices;
	/// True if the output indices have been cached.
	bool cachedOutputIndices;

private:
	/// Given all the timers, return just the ones that provide inputs.
	/// \param timers The timers.
	/// \return The timers that provide inputs.
	const std::unordered_set<int> getTimerInputChannels(const std::unordered_map<int, LabJack::TimerMode>& timers) const
	{
		std::unordered_set<int> timersWithInputs;
		for (auto timer = timers.cbegin(); timer != timers.cend(); ++timer)
		{
			if ((timer->second != LabJack::TIMERMODE_PWM_16BIT) &&
				(timer->second != LabJack::TIMERMODE_PWM_8BIT) &&
				(timer->second != LabJack::TIMERMODE_FREQUENCY_OUTPUT))
			{
				timersWithInputs.insert(timer->first);
			}
		}
		return timersWithInputs;
	}

	/// Given all the timers, return just the ones that take outputs.
	/// \param timers The timers.
	/// \return The timers that take outputs.
	const std::unordered_set<int> getTimerOutputChannels(const std::unordered_map<int, LabJack::TimerMode>& timers) const
	{
		std::unordered_set<int> timersWithOutputs;
		for (auto timer = timers.cbegin(); timer != timers.cend(); ++timer)
		{
			if ((timer->second != LabJack::TIMERMODE_PWM_16BIT) &&
				(timer->second != LabJack::TIMERMODE_PWM_8BIT) &&
				(timer->second != LabJack::TIMERMODE_RISING_EDGES_32BIT) &&
				(timer->second != LabJack::TIMERMODE_FALLING_EDGES_32BIT) &&
				(timer->second != LabJack::TIMERMODE_DUTY_CYCLE) &&
				(timer->second != LabJack::TIMERMODE_FIRMWARE_COUNTER) &&
				(timer->second != LabJack::TIMERMODE_FIRMWARE_COUNTER_DEBOUNCED) &&
				(timer->second != LabJack::TIMERMODE_FREQUENCY_OUTPUT) &&
				(timer->second != LabJack::TIMERMODE_QUADRATURE) &&
				(timer->second != LabJack::TIMERMODE_RISING_EDGES_16BIT) &&
				(timer->second != LabJack::TIMERMODE_FALLING_EDGES_16BIT) &&
				(timer->second != LabJack::TIMERMODE_LINE_TO_LINE))
			{
				timersWithOutputs.insert(timer->first);
			}
		}
		return timersWithOutputs;
	}

	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;
};

/// The per-scaffold data (in comparison to DeviceData the per-device data).
/// Note that there is only a single instance of LabJackScaffold and so only a single instance of this struct.
struct LabJackScaffold::StateData
{
public:
	/// Initialize the state.
	StateData()
	{
	}

	/// The list of known devices.
	std::list<std::unique_ptr<LabJackScaffold::DeviceData>> activeDeviceList;

	/// The mutex that protects the list of known devices.
	boost::mutex mutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	StateData(const StateData&) /*= delete*/;
	StateData& operator=(const StateData&) /*= delete*/;
};

LabJackScaffold::LabJackScaffold() :
	m_state(new StateData)
{
	m_logger = SurgSim::Framework::Logger::getLogger("LabJack device");
	SURGSIM_LOG_DEBUG(m_logger) << "Shared scaffold created.  LabJackUD driver version: " <<
		GetDriverVersion() << ".";
}

LabJackScaffold::~LabJackScaffold()
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	if (!m_state->activeDeviceList.empty())
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Destroying scaffold while devices are active!?!";
		for (auto it = m_state->activeDeviceList.begin();  it != m_state->activeDeviceList.end();  ++it)
		{
			if ((*it)->thread)
			{
				destroyPerDeviceThread(it->get());
			}
		}
		m_state->activeDeviceList.clear();
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Shared scaffold destroyed.";
}

bool LabJackScaffold::registerDevice(LabJackDevice* device)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	bool result = true;

	// Make sure the object is unique.
	auto sameObject = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	SURGSIM_ASSERT(sameObject == m_state->activeDeviceList.end()) << "LabJack: Tried to register a device named '" <<
		device->getName() << "', which is already present!";

	// Make sure the name is unique.
	const std::string& deviceName = device->getName();
	auto const sameName = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[&deviceName](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getName() == deviceName; });
	if (sameName != m_state->activeDeviceList.end())
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Tried to register a device when the same name, '" <<
			device->getName() << "', is already present!";
		result = false;
	}

	// Make sure the combination of connection and address is unique, unless the address is zero-length, in which
	// case the first-found device of this model on this connection will be opened.
	const std::string& address = device->getAddress();
	if (result && (address.length() > 0))
	{
		const LabJack::Model model = device->getModel();
		const LabJack::Connection connection = device->getConnection();

		auto const sameInitialization = std::find_if(m_state->activeDeviceList.cbegin(),
			m_state->activeDeviceList.cend(),
			[&address, connection, model](const std::unique_ptr<DeviceData>& info)
		{ return (info->deviceObject->getAddress() == address) &&
				(info->deviceObject->getConnection() == connection) &&
				(info->deviceObject->getModel() == model); });

		if (sameInitialization != m_state->activeDeviceList.cend())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Tried to register a device named '" << device->getName() <<
				"', but a device with the same model (" << model << "), connection (" << connection <<
				"), and address ('" << address << "') is already present!";
			result = false;
		}
	}

	if (result)
	{
		// Create a handle, opening communications.
		// If the device's model or connection are SEARCH, iterate over the options.
		std::vector<LabJack::Model> modelsToSearch;
		if (device->getModel() == LabJack::MODEL_SEARCH)
		{
			SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << ": searching for models U3, U6, and UE9.";
			modelsToSearch.push_back(LabJack::MODEL_U6);
			modelsToSearch.push_back(LabJack::MODEL_U3);
			modelsToSearch.push_back(LabJack::MODEL_UE9);
		}
		else
		{
			modelsToSearch.push_back(device->getModel());
		}

		std::vector<LabJack::Connection> connectionsToSearch;
		if (device->getConnection() == LabJack::CONNECTION_SEARCH)
		{
			SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() <<
				": searching for connections USB and Ethernet.";
			connectionsToSearch.push_back(LabJack::CONNECTION_USB);
			connectionsToSearch.push_back(LabJack::CONNECTION_ETHERNET);
		}
		else
		{
			connectionsToSearch.push_back(device->getConnection());
		}

		std::unique_ptr<Handle> handle;
		for (auto model = modelsToSearch.cbegin(); model != modelsToSearch.cend(); ++model)
		{
			for (auto connection = connectionsToSearch.cbegin(); connection != connectionsToSearch.cend(); ++connection)
			{
				device->setModel(*model);
				device->setConnection(*connection);

				handle = std::unique_ptr<Handle>(new Handle(*model, *connection, address));
				result = handle->isValid();
				if (result)
				{
					auto const sameHandle = std::find_if(m_state->activeDeviceList.cbegin(),
						m_state->activeDeviceList.cend(),
						[&handle](const std::unique_ptr<DeviceData>& info)
						{ return (info->deviceHandle->get() == handle->get()); });

					if (sameHandle != m_state->activeDeviceList.cend())
					{
						SURGSIM_LOG_INFO(m_logger) << "Tried to register a device named '" << device->getName() <<
							"', but a device with the same handle (" << handle->get() << ") is already present!  " <<
							"This can happen if multiple LabJack devices are used without setting their addresses.";
						handle->destroy(); // The handle was initialized and will be destructed.
						result = false;
					}
					else
					{
						break;
					}
				}
			}
			if (result)
			{
				break;
			}
		}

		if (result)
		{
			std::unique_ptr<DeviceData> info(new DeviceData(device, std::move(handle)));

			// Cache the NamedData indices for the input DataGroup.
			const SurgSim::DataStructures::DataGroup& inputData = device->getInputData();

			for (auto input = info->digitalInputChannels.cbegin();
				input != info->digitalInputChannels.cend();
				++input)
			{
				const std::string name = SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX + std::to_string(*input);
				info->digitalInputIndices[*input] = inputData.scalars().getIndex(name);
				SURGSIM_ASSERT(info->digitalInputIndices[*input] >= 0) << "LabJackScaffold::DeviceData " <<
					"failed to get a valid NamedData index for the digital input for line " << *input <<
					".  Make sure that is a valid line number.  Expected an entry named " << name << ".";
			}

			for (auto timer = info->timerInputChannels.cbegin();
				timer != info->timerInputChannels.cend();
				++timer)
			{
				const std::string name = SurgSim::DataStructures::Names::TIMER_INPUT_PREFIX + std::to_string(*timer);
				info->timerInputIndices[*timer] = inputData.scalars().getIndex(name);
				SURGSIM_ASSERT(info->timerInputIndices[*timer] >= 0) << "LabJackScaffold::DeviceData " <<
					"failed to get a valid NamedData index for the timer for channel " << *timer <<
					".  Make sure that is a valid timer number.  Expected an entry named " << name << ".";
			}

			for (auto input = info->analogInputsDifferential.cbegin();
				input != info->analogInputsDifferential.cend(); ++input)
			{
				std::string name = SurgSim::DataStructures::Names::ANALOG_INPUT_DIFFERENTIAL_PREFIX +
					std::to_string(input->first);
				info->analogInputDifferentialIndices[input->first] = inputData.scalars().getIndex(name);
				SURGSIM_ASSERT(info->analogInputDifferentialIndices[input->first] >= 0) <<
					"LabJackScaffold::DeviceData failed to get a valid NamedData index for the differential " <<
					"analog input for channel " << input->first << ".  Make sure that is a valid line number.  " <<
					"Expected an entry named " << name << ".";
			}

			for (auto input = info->analogInputsSingleEnded.cbegin();
				input != info->analogInputsSingleEnded.cend(); ++input)
			{
				const std::string name = SurgSim::DataStructures::Names::ANALOG_INPUT_SINGLE_ENDED_PREFIX +
					std::to_string(input->first);
				info->analogInputSingleEndedIndices[input->first] = inputData.scalars().getIndex(name);
				SURGSIM_ASSERT(info->analogInputSingleEndedIndices[input->first] >= 0) <<
					"LabJackScaffold::DeviceData failed to get a valid NamedData index for the single-ended " <<
					"analog input for line " << input->first << ".  Make sure that is a valid line number.  " <<
					"Expected an entry named " << name << ".";
			}

			std::unique_ptr<LabJackThread> thread(new LabJackThread(this, info.get()));
			thread->setRate(device->getMaximumUpdateRate());
			thread->start();

			info.get()->thread = std::move(thread);
			m_state->activeDeviceList.emplace_back(std::move(info));
		}
	}

	return result;
}

bool LabJackScaffold::unregisterDevice(const LabJackDevice* const device)
{
	bool found = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
		if (matching != m_state->activeDeviceList.end())
		{
			if ((*matching)->thread)
			{
				destroyPerDeviceThread(matching->get());
				matching->get()->deviceHandle->destroy();
			}
			m_state->activeDeviceList.erase(matching);
			// the iterator is now invalid but that's OK
			found = true;
		}
	}

	if (!found)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Attempted to release a non-registered device named '" <<
			device->getName() << ".";
	}
	return found;
}

bool LabJackScaffold::runInputFrame(LabJackScaffold::DeviceData* info)
{
	if (info->deviceObject->pullOutput() && !info->cachedOutputIndices)
	{
		const SurgSim::DataStructures::DataGroup& initialOutputData = info->deviceObject->getOutputData();

		const std::unordered_set<int>& digitalOutputChannels = info->digitalOutputChannels;
		for (auto output = digitalOutputChannels.cbegin(); output != digitalOutputChannels.cend(); ++output)
		{
			info->digitalOutputIndices[*output] =
				initialOutputData.scalars().getIndex(SurgSim::DataStructures::Names::DIGITAL_OUTPUT_PREFIX +
				std::to_string(*output));
		}

		const std::unordered_set<int>& timerOutputChannels = info->timerOutputChannels;
		for (auto timer = timerOutputChannels.cbegin(); timer != timerOutputChannels.cend(); ++timer)
		{
			info->timerOutputIndices[*timer] =
				initialOutputData.scalars().getIndex(SurgSim::DataStructures::Names::TIMER_OUTPUT_PREFIX +
				std::to_string(*timer));
		}

		const std::unordered_set<int>& analogOutputChannels = info->analogOutputChannels;
		for (auto output = analogOutputChannels.cbegin(); output != analogOutputChannels.cend(); ++output)
		{
			info->analogOutputIndices[*output] =
				initialOutputData.scalars().getIndex(SurgSim::DataStructures::Names::ANALOG_OUTPUT_PREFIX +
				std::to_string(*output));
		}

		info->cachedOutputIndices = true;
	}

	if (!info->deviceObject->hasOutputProducer() || info->cachedOutputIndices)
	{
		if (!updateDevice(info))
		{
			return false;
		}
		info->deviceObject->pushInput();
	}
	return true;
}

bool LabJackScaffold::updateDevice(LabJackScaffold::DeviceData* info)
{
	const LJ_HANDLE rawHandle = info->deviceHandle->get();

	// First we AddRequests.  This clears the old data.
	const SurgSim::DataStructures::DataGroup& outputData = info->deviceObject->getOutputData();

	// Request the values of digital inputs.
	const std::unordered_set<int>& digitalInputChannels = info->digitalInputChannels;
	for (auto input = digitalInputChannels.cbegin(); input != digitalInputChannels.cend(); ++input)
	{
		const LJ_ERROR error = AddRequest(rawHandle, LJ_ioGET_DIGITAL_BIT, *input, 0, 0, 0);
		SURGSIM_LOG_IF(!isOk(error), m_logger, WARNING) <<
			"Failed to request digital input for a device named '" << info->deviceObject->getName() <<
			"', line number " << *input << "." << std::endl << formatErrorMessage(error);
	}

	// Request to set digital outputs.
	const std::unordered_set<int>& digitalOutputChannels = info->digitalOutputChannels;
	for (auto output = digitalOutputChannels.cbegin(); output != digitalOutputChannels.cend(); ++output)
	{
		if (info->digitalOutputIndices.count(*output) > 0)
		{
			const int index = info->digitalOutputIndices[*output];
			SURGSIM_ASSERT(index >= 0) << "LabJackScaffold: A LabJackDevice was configured with line " << *output <<
				" set to digital output, but the scaffold does not know the correct index into the NamedData. " <<
				" Make sure there is an entry in the scalars with the correct string key.";

			double value;
			if (outputData.scalars().get(index, &value))
			{
				const LJ_ERROR error = AddRequest(rawHandle, LJ_ioPUT_DIGITAL_BIT, *output, value, 0, 0);
				SURGSIM_LOG_IF(!isOk(error), m_logger, WARNING) <<
					"Failed to set digital output for a device named '" << info->deviceObject->getName() <<
					"', line number " << *output << ", value " << value << "." <<
					std::endl << formatErrorMessage(error);
			}
		}
	}

	// Request to set to timers (e.g., resetting firmware counters).
	const std::unordered_set<int>& timerOutputChannels = info->timerOutputChannels;
	for (auto timer = timerOutputChannels.cbegin(); timer != timerOutputChannels.cend(); ++timer)
	{
		if (info->timerOutputIndices.count(*timer) > 0)
		{
			const int index = info->timerOutputIndices[*timer];
			// We do not ensure that all the timers can be output to, because the user might not need to reset them.
			if (index >= 0)
			{
				double value;
				if (outputData.scalars().get(index, &value))
				{
					const LJ_ERROR error = AddRequest(rawHandle, LJ_ioPUT_TIMER_VALUE, *timer, value, 0, 0);
					SURGSIM_LOG_IF(!isOk(error), m_logger, WARNING) <<
						"Failed to set timer value for a device named '" << info->deviceObject->getName() <<
						"', channel number " << *timer << ", value " << value << "." <<
						std::endl << formatErrorMessage(error);
				}
			}
		}
	}

	// Request inputs from timers.
	const std::unordered_set<int>& timerInputChannels = info->timerInputChannels;
	for (auto timer = timerInputChannels.cbegin(); timer != timerInputChannels.cend(); ++timer)
	{
		const LJ_ERROR error = AddRequest(rawHandle, LJ_ioGET_TIMER, *timer, 0, 0, 0);
		SURGSIM_LOG_IF(!isOk(error), m_logger, WARNING) <<
			"Failed to request timer input for a device named '" << info->deviceObject->getName() <<
			"', channel number " << *timer << "." << std::endl << formatErrorMessage(error);
	}

	// Request the values of analog inputs.
	auto const& analogInputsDifferential = info->analogInputsDifferential;
	for (auto input = analogInputsDifferential.cbegin(); input != analogInputsDifferential.cend(); ++input)
	{
		const LJ_ERROR error = AddRequest(rawHandle, LJ_ioGET_AIN_DIFF, input->first, 0,
			input->second.negativeChannel.getValue(), 0);
		SURGSIM_LOG_IF(!isOk(error), m_logger, WARNING) <<
			"Failed to request differential analog input for a device named '" << info->deviceObject->getName() <<
			"', positive channel " << input->first << ", negative channel " <<
			input->second.negativeChannel.getValue() << "." << std::endl << formatErrorMessage(error);
	}

	auto const& analogInputsSingleEnded = info->analogInputsSingleEnded;
	for (auto input = analogInputsSingleEnded.cbegin(); input != analogInputsSingleEnded.cend(); ++input)
	{
		const LJ_ERROR error = AddRequest(rawHandle, LJ_ioGET_AIN, input->first, 0, 0, 0);
		SURGSIM_LOG_IF(!isOk(error), m_logger, WARNING) <<
			"Failed to request single-ended analog input for a device named '" << info->deviceObject->getName() <<
			"', channel " << input->first << "." << std::endl << formatErrorMessage(error);
	}

	// Request to set analog outputs.
	const std::unordered_set<int>& analogOutputChannels = info->analogOutputChannels;
	for (auto output = analogOutputChannels.cbegin(); output != analogOutputChannels.cend(); ++output)
	{
		if (info->analogOutputIndices.count(*output) > 0)
		{
			const int index = info->analogOutputIndices[*output];
			SURGSIM_ASSERT(index >= 0) << "LabJackScaffold: A LabJackDevice was configured with line " << *output <<
				" set to analog output, but the scaffold does not know the correct index into the NamedData. " <<
				" Make sure there is an entry in the scalars with the correct string key.";

			double value;
			if (outputData.scalars().get(index, &value))
			{
				const LJ_ERROR error = AddRequest(rawHandle, LJ_ioPUT_DAC, *output, value, 0, 0);
				SURGSIM_LOG_IF(!isOk(error), m_logger, WARNING) <<
					"Failed to set analog output for a device named '" << info->deviceObject->getName() <<
					"', line number " << *output << ", value " << value << "." <<
					std::endl << formatErrorMessage(error);
			}
		}
	}

	// GoOne, telling this specific LabJack to perform the requests.
	const LJ_ERROR error = GoOne(rawHandle);

	// Finally we get the results.
	SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();
	if (isOk(error))
	{
		// Digital inputs.
		for (auto input = digitalInputChannels.cbegin(); input != digitalInputChannels.cend(); ++input)
		{
			double value;
			const LJ_ERROR error = GetResult(rawHandle, LJ_ioGET_DIGITAL_BIT, *input, &value);
			if (isOk(error))
			{
				inputData.scalars().set(info->digitalInputIndices[*input], value);
			}
			else
			{
				SURGSIM_LOG_WARNING(m_logger) << "Failed to get digital input for a device named '" <<
					info->deviceObject->getName() << "', line number " << *input << "." << std::endl <<
					formatErrorMessage(error);
				inputData.scalars().reset(info->digitalInputIndices[*input]);
			}
		}

		// Timer inputs.
		for (auto timer = timerInputChannels.cbegin(); timer != timerInputChannels.cend(); ++timer)
		{
			double value;
			const LJ_ERROR error = GetResult(rawHandle, LJ_ioGET_TIMER, *timer, &value);
			if (isOk(error))
			{
				inputData.scalars().set(info->timerInputIndices[*timer], value);
			}
			else
			{
				SURGSIM_LOG_WARNING(m_logger) << "Failed to get timer input for a device named '" <<
					info->deviceObject->getName() << "', channel number " << *timer << "." << std::endl <<
					formatErrorMessage(error);
				inputData.scalars().reset(info->timerInputIndices[*timer]);
			}
		}

		// Analog inputs.
		for (auto input = analogInputsDifferential.cbegin(); input != analogInputsDifferential.cend(); ++input)
		{
			double value;
			const LJ_ERROR error = GetResult(rawHandle, LJ_ioGET_AIN_DIFF, input->first, &value);
			if (isOk(error))
			{
				inputData.scalars().set(info->analogInputDifferentialIndices[input->first], value);
			}
			else
			{
				SURGSIM_LOG_WARNING(m_logger) << "Failed to get differential analog input for a device named '" <<
					info->deviceObject->getName() << "', positive channel " << input->first << ", negative channel " <<
					input->second.negativeChannel.getValue() << "." << std::endl << formatErrorMessage(error);
				inputData.scalars().reset(info->analogInputDifferentialIndices[input->first]);
			}
		}

		for (auto input = analogInputsSingleEnded.cbegin(); input != analogInputsSingleEnded.cend(); ++input)
		{
			double value;
			const LJ_ERROR error = GetResult(rawHandle, LJ_ioGET_AIN, input->first, &value);
			if (isOk(error))
			{
				inputData.scalars().set(info->analogInputSingleEndedIndices[input->first], value);
			}
			else
			{
				SURGSIM_LOG_WARNING(m_logger) << "Failed to get single-ended analog input for a device named '" <<
					info->deviceObject->getName() << "', channel " << input->first << "." << std::endl <<
					formatErrorMessage(error);
				inputData.scalars().reset(info->analogInputSingleEndedIndices[input->first]);
			}
		}
	}
	else
	{
		SURGSIM_LOG_WARNING(m_logger) << "Failed to submit requests for a device named '" <<
			info->deviceObject->getName() << "." << std::endl << formatErrorMessage(error);
		inputData.resetAll();
	}

	return true;
}

bool LabJackScaffold::destroyPerDeviceThread(DeviceData* data)
{
	SURGSIM_ASSERT(data->thread) << "LabJack: destroying a per-device thread, but none exists for this DeviceData";

	std::unique_ptr<LabJackThread> thread = std::move(data->thread);
	thread->stop();
	thread.reset();

	return true;
}

SurgSim::DataStructures::DataGroup LabJackScaffold::buildDeviceInputData()
{
	SurgSim::DataStructures::DataGroupBuilder builder;
	// We don't know which input lines we need until after the configuration, but LabJackDevice must be constructed with
	// a valid DataGroup, so we add every possible line.
	const int maxDigitalInputs = 23; // The UE9 can have 23 digital inputs.
	for (int i = 0; i < maxDigitalInputs; ++i)
	{
		builder.addScalar(SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX + std::to_string(i));
	}

	const int maxTimerInputs = 6; // The UE9 can have 6 timers.
	for (int i = 0; i < maxTimerInputs; ++i)
	{
		builder.addScalar(SurgSim::DataStructures::Names::TIMER_INPUT_PREFIX + std::to_string(i));
	}

	const int maxAnalogInputs = 16; // The U3 can have 16 analog inputs.
	for (int i = 0; i < maxAnalogInputs; ++i)
	{
		builder.addScalar(SurgSim::DataStructures::Names::ANALOG_INPUT_DIFFERENTIAL_PREFIX + std::to_string(i));
		builder.addScalar(SurgSim::DataStructures::Names::ANALOG_INPUT_SINGLE_ENDED_PREFIX + std::to_string(i));
	}
	return builder.createData();
}

std::shared_ptr<LabJackScaffold> LabJackScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<LabJackScaffold> sharedInstance;
	return sharedInstance.get();
}

bool LabJackScaffold::configureDevice(DeviceData* deviceData)
{
	LJ_HANDLE rawHandle = deviceData->deviceHandle->get();

	// Reset the configuration.
	LJ_ERROR error = ePut(rawHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	bool result = isOk(error);
	SURGSIM_LOG_IF(!result, m_logger, SEVERE) <<
		"Failed to reset configuration for a device named '" << deviceData->deviceObject->getName() << "." <<
		std::endl << formatErrorMessage(error);

	return result && configureClockAndTimers(deviceData) && configureDigital(deviceData) && configureAnalog(deviceData);
}

bool LabJackScaffold::configureClockAndTimers(DeviceData* deviceData)
{
	bool result = configureNumberOfTimers(deviceData);

	if (result && (deviceData->deviceObject->getTimers().size() > 0))
	{
		result = configureClock(deviceData) && configureTimers(deviceData);
	}
	return result;
}

bool LabJackScaffold::configureNumberOfTimers(DeviceData* deviceData)
{
	LabJackDevice* device = deviceData->deviceObject;
	LJ_HANDLE rawHandle = deviceData->deviceHandle->get();

	const std::unordered_map<int, LabJack::TimerMode>& timers = device->getTimers();

	LJ_ERROR error = ePut(rawHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, timers.size(), 0);
	bool result = isOk(error);
	SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
		"Failed to configure number of enabled timers for a device named '" << device->getName() <<
		"', with number of timers " << timers.size() << "." << std::endl << formatErrorMessage(error);

	// Counters are not yet supported so they are explicitly disabled.
	const int numberOfChannels = 2; // The LabJack U3, U6, and UE9 models each have two counters.
	for (int channel = 0; channel < numberOfChannels; ++channel)
	{
		const int enable = 0; // Disable both counters.
		const LJ_ERROR error = ePut(rawHandle, LJ_ioPUT_COUNTER_ENABLE, channel, enable, 0);
		result = result && isOk(error);
		SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
			"Failed to enable/disable counter for a device named '" << device->getName() << "." <<
			std::endl << formatErrorMessage(error);
	}

	error = ePut(rawHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_COUNTER_PIN_OFFSET, device->getTimerCounterPinOffset(), 0);
	result = result && isOk(error);
	SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
		"Failed to configure timer/counter pin offset for a device named '" << device->getName() <<
		"', with offset " << device->getTimerCounterPinOffset() << "." << std::endl << formatErrorMessage(error);

	return result;
}

bool LabJackScaffold::configureClock(DeviceData* deviceData)
{
	LabJackDevice* device = deviceData->deviceObject;
	LJ_HANDLE rawHandle = deviceData->deviceHandle->get();

	LabJack::TimerBase base = device->getTimerBase();
	if (base == LabJack::TIMERBASE_DEFAULT)
	{
		LabJackDefaults defaults;
		base = defaults.timerBase[device->getModel()];
	}
	LJ_ERROR error = ePut(rawHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, base, 0);
	bool result = isOk(error);
	SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
		"Failed to configure the timer base rate for a device named '" << device->getName() <<
		"', with timer base " << device->getTimerBase() << "." << std::endl << formatErrorMessage(error);

	error = ePut(rawHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, device->getTimerClockDivisor(), 0);
	result = result && isOk(error);
	SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
		"Failed to configure the timer/clock divisor for a device named '" << device->getName() <<
		"', with divisor " << device->getTimerClockDivisor() << "." << std::endl << formatErrorMessage(error);

	return result;
}

bool LabJackScaffold::configureTimers(DeviceData* deviceData)
{
	LabJackDevice* device = deviceData->deviceObject;
	LJ_HANDLE rawHandle = deviceData->deviceHandle->get();

	bool result = true;

	const std::unordered_map<int, LabJack::TimerMode>& timers = device->getTimers();
	for (auto timer = timers.cbegin(); timer != timers.cend(); ++timer)
	{
		LJ_ERROR error = ePut(rawHandle, LJ_ioPUT_TIMER_MODE, timer->first, timer->second, 0);
		result = result && isOk(error);
		SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
			"Failed to configure a timer for a device named '" << device->getName() <<
			"', timer number " << timer->first << ", with mode code " << timer->second << "." <<
			std::endl << formatErrorMessage(error);
		if (result &&
			((timer->second == LabJack::TimerMode::TIMERMODE_PWM_8BIT) ||
			(timer->second == LabJack::TimerMode::TIMERMODE_PWM_16BIT)))
		{  // Initialize PWMs to almost-always low.
			const int value = 65535; // the value corresponding to a PWM that is low as much as possible.
			error = ePut(rawHandle, LJ_ioPUT_TIMER_VALUE, timer->first, value, 0);
			result = result && isOk(error);
			SURGSIM_LOG_IF(!result, m_logger, SEVERE) <<
				"Failed to set the initial value for a PWM timer for a device named '" << device->getName() <<
				"', timer number " << timer->first << ", with mode code " << timer->second <<
				", and value " << value << "."  << std::endl << formatErrorMessage(error);
		}
	}

	return result;
}

bool LabJackScaffold::configureAnalog(DeviceData* deviceData)
{
	LabJackDevice* device = deviceData->deviceObject;
	LJ_HANDLE rawHandle = deviceData->deviceHandle->get();

	bool result = true;

	const std::unordered_set<int>& analogOutputs = device->getAnalogOutputChannels();
	for (auto output = analogOutputs.cbegin(); output != analogOutputs.cend(); ++output)
	{
		LJ_ERROR error = ePut(rawHandle, LJ_ioPUT_DAC_ENABLE, *output, 1, 0);
		result = result && isOk(error);
		SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
			"Failed to enable analog output for a device named '" << device->getName() <<
			"', channel " << *output << "." << std::endl << formatErrorMessage(error);
	}

	auto const& analogInputsDifferential = device->getAnalogInputsDifferential();
	auto const& analogInputsSingleEnded = device->getAnalogInputsSingleEnded();
	if ((analogInputsDifferential.size() > 0) || (analogInputsSingleEnded.size() > 0))
	{
		LJ_ERROR error = ePut(rawHandle, LJ_ioPUT_CONFIG, LJ_chAIN_RESOLUTION, device->getAnalogInputResolution(), 0);
		result = result && isOk(error);
		SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
			"Failed to configure analog input resolution for a device named '" << device->getName() <<
			"', with resolution code " << device->getAnalogInputResolution() << "." << std::endl <<
			formatErrorMessage(error);

		error = ePut(rawHandle, LJ_ioPUT_CONFIG, LJ_chAIN_SETTLING_TIME, device->getAnalogInputSettling(), 0);
		result = result && isOk(error);
		SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
			"Failed to configure analog input settling time for a device named '" << device->getName() <<
			"', with settling time code " << device->getAnalogInputSettling() << "." << std::endl <<
			formatErrorMessage(error);

		for (auto input = analogInputsDifferential.cbegin(); input != analogInputsDifferential.cend(); ++input)
		{
			error = ePut(rawHandle, LJ_ioPUT_ANALOG_ENABLE_BIT, input->first, 1, 0);
			result = result && isOk(error);
			SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
				"Failed to enable analog input for a device named '" << device->getName() <<
				"', channel " << input->first << "." << std::endl << formatErrorMessage(error);

			error = ePut(rawHandle, LJ_ioPUT_AIN_RANGE, input->first, input->second.range, 0);
			result = result && isOk(error);
			SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
				"Failed to set the range for an analog input for a device named '" << device->getName() <<
				"', channel " << input->first << ", with range code " << input->second.range << "." <<
				std::endl << formatErrorMessage(error);
		}

		for (auto input = analogInputsSingleEnded.cbegin(); input != analogInputsSingleEnded.cend(); ++input)
		{
			error = ePut(rawHandle, LJ_ioPUT_ANALOG_ENABLE_BIT, input->first, 1, 0);
			result = result && isOk(error);
			SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
				"Failed to enable analog input for a device named '" << device->getName() <<
				"', channel " << input->first << "." << std::endl << formatErrorMessage(error);

			error = ePut(rawHandle, LJ_ioPUT_AIN_RANGE, input->first, input->second, 0);
			result = result && isOk(error);
			SURGSIM_LOG_IF(!isOk(error), m_logger, SEVERE) <<
				"Failed to set the range for an analog input for a device named '" << device->getName() <<
				"', channel " << input->first << ", with range code " << input->second << "." <<
				std::endl << formatErrorMessage(error);
		}
	}

	return result;
}

bool LabJackScaffold::configureDigital(DeviceData* deviceData)
{
	return true;
}

std::shared_ptr<SurgSim::Framework::Logger> LabJackScaffold::getLogger() const
{
	return m_logger;
}

};  // namespace Device
};  // namespace SurgSim

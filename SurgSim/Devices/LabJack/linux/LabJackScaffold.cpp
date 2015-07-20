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
#include <array>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <errno.h>
#include <labjackusb.h> // the low-level LabJack library (aka exodriver)
#include <list>
#include <memory>
#include <stdint.h> // fixed-width integer types are used for low-level data communication
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/LabJack/LabJackDevice.h"
#include "SurgSim/Devices/LabJack/LabJackThread.h"
#include "SurgSim/Devices/LabJack/linux/LabJackChecksums.h"
#include "SurgSim/Devices/LabJack/linux/LabJackConstants.h"
#include "SurgSim/Devices/LabJack/linux/LabJackTypeConverters.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"

namespace SurgSim
{
namespace Device
{

namespace
{
/// Distinguish the name of the hardware library's handle from our wrapper with a prefix.
typedef HANDLE LJ_HANDLE;

/// The low-level LabJack library returns a NULL handle when failing to open a new device.
static const LJ_HANDLE LABJACK_INVALID_HANDLE = NULL;

/// A struct containing the default settings that depend on the model of LabJack.
struct LabJackDefaults
{
	LabJackDefaults()
	{
		timerBase[LabJack::MODEL_U3] = LabJack::TIMERBASE_2;
		timerBase[LabJack::MODEL_U6] = LabJack::TIMERBASE_2;
		timerBase[LabJack::MODEL_UE9] = LabJack::TIMERBASE_1;
	}

	/// The default timer base rate.
	std::unordered_map<LabJack::Model, LabJack::TimerBase, std::hash<int>> timerBase;
};


/// A struct containing various parameters that depend on the model of LabJack.
struct LabJackParameters
{
	LabJackParameters()
	{
		configBlocks[LabJack::MODEL_U3] = 16; // hardware version 1.30 & 1.21
		configBlocks[LabJack::MODEL_U6] = 10;
		configBlocks[LabJack::MODEL_UE9] = 4;

		calibrationCommand[LabJack::MODEL_U3] = 0x2D;
		calibrationCommand[LabJack::MODEL_U6] = 0x2D;
		calibrationCommand[LabJack::MODEL_UE9] = 0x2A;

		calibrationThirdByte[LabJack::MODEL_U3] = 0x11;
		calibrationThirdByte[LabJack::MODEL_U6] = 0x11;
		calibrationThirdByte[LabJack::MODEL_UE9] = 0x41;

		calibrationReadBytes[LabJack::MODEL_U3] = 40;
		calibrationReadBytes[LabJack::MODEL_U6] = 40;
		calibrationReadBytes[LabJack::MODEL_UE9] = 136;
	}

	/// The number of config memory blocks.
	std::unordered_map<LabJack::Model, int, std::hash<int>> configBlocks;
	/// The extended command for reading the calibration data.
	std::unordered_map<LabJack::Model, BYTE, std::hash<int>> calibrationCommand;
	/// The expected read third byte for the read calibration command.
	std::unordered_map<LabJack::Model, BYTE, std::hash<int>> calibrationThirdByte;
	/// The number of bytes read per calibration read.
	std::unordered_map<LabJack::Model, int, std::hash<int>> calibrationReadBytes;
};

/// Convert the LabJack::Range to the gain code expected by the low level driver.
/// \param range The LabJack::Range code.
/// \return The gain code.
int getGain(LabJack::Range range)
{
	int gain;
	switch (range)
	{
	case LabJack::RANGE_10:
		gain = 0;
		break;
	case LabJack::RANGE_1:
		gain = 1;
		break;
	case LabJack::RANGE_0_POINT_1:
		gain = 2;
		break;
	case LabJack::RANGE_0_POINT_01:
		gain = 3;
		break;
	default:
		gain = 15;
	}
	return gain;
}

/// Read from the LabJack and do most of the checksum tests.  This function does not test readBytes[2] because its
/// expected value varies.
/// \param rawHandle The handle.
/// \param readBytes The array of bytes.
/// \param readBytesSize The number of bytes that should be read.
/// \param name The name of the device being read.
/// \param sendBytes The bytes that were sent.
/// \param text The text explaining what the read is for, used in error log messages.
/// \param logger The logger for error messages.
/// \return true if no errors.
bool readAndCheck(const LJ_HANDLE rawHandle, std::array<BYTE, LabJack::MAXIMUM_BUFFER>* readBytes,
				  int readBytesSize, const std::string& name,
				  const std::array<BYTE, LabJack::MAXIMUM_BUFFER>& sendBytes, const std::string& text,
				  std::shared_ptr<SurgSim::Framework::Logger> logger)
{
	bool result = true;

	const int read = LJUSB_Read(rawHandle, &((*readBytes)[0]), readBytesSize);
	const uint16_t checksumTotal = LabJack::extendedChecksum16(*readBytes, readBytesSize);
	if (read < readBytesSize)
	{
		SURGSIM_LOG_SEVERE(logger) << "Failed to read response of " << text << " a device named '" << name << "'. "
			<< readBytesSize << " bytes were expected, but only " << read << " were received." << std::endl <<
			"  labjackusb error code: " << errno << "." << std::endl;
		result = false;
	}
	else if ((((checksumTotal / 256 ) & 0xff) != (*readBytes)[5]) ||
		((checksumTotal & 0xff) != (*readBytes)[4]) ||
		(LabJack::extendedChecksum8(*readBytes) != (*readBytes)[0]))
	{
		SURGSIM_LOG_SEVERE(logger) << "Failed to read response of " << text << " a device named '" << name <<
			"'.  The checksums are bad." << std::endl << "  labjackusb error code: " << errno << "." << std::endl;
		result = false;
	}
	else if (((*readBytes)[1] != sendBytes[1]) || ((*readBytes)[3] != sendBytes[3]))
	{
		SURGSIM_LOG_SEVERE(logger) << "Failed to read response of " << text << " a device named '" << name <<
			"'.  The command bytes are wrong.  Expected bytes 1 & 3: " << static_cast<int>(sendBytes[1]) << ","
			<< static_cast<int>(sendBytes[3]) << ".  Received bytes 1 & 3: " << static_cast<int>((*readBytes)[1]) <<
			", " << static_cast<int>((*readBytes)[3]) << "." << std::endl <<
			"  labjackusb error code: " << errno << "." << std::endl;
		result = false;
	}
	else if ((*readBytes)[6] != 0)
	{
		SURGSIM_LOG_SEVERE(logger) << "Failed to read response of " << text << " a device named '" << name <<
			"'.  The device library returned an error code: " << static_cast<int>((*readBytes)[6]) << ", for frame: " <<
			static_cast<int>((*readBytes)[7]) << std::endl << "  labjackusb error code: " << errno << "." << std::endl;
		result = false;
	}

	return result;
}

};

class LabJackScaffold::Handle
{
public:
	/// Constructor that attempts to open a device.
	/// \param model The model of LabJack device to open (see strings in LabJackUD.h).
	/// \param connection How to connect to the device (e.g., USB) (see strings in LabJackUD.h).
	/// \param address Either the ID or serial number (if USB), or the IP address.
	Handle(SurgSim::Device::LabJack::Model model, SurgSim::Device::LabJack::Connection connection,
		const std::string& address) :
		m_deviceHandle(LABJACK_INVALID_HANDLE),
		m_address(address),
		m_model(model),
		m_connection(connection),
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

		bool result = true;

		if (m_model == LabJack::MODEL_UE9)
		{
			SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Failed to open a device. " <<
				"The UE9 model LabJack is not supported for the low-level driver used on Linux & Mac. " <<
				"The commands for the UE9 have a different structure, which is not currently implemented." <<
				std::endl <<
				"  Model: '" << m_model << "'.  Connection: '" << m_connection << "'.  Address: '" <<
				m_address << "'." << std::endl;
			result = false;
		}

		if (m_connection != LabJack::CONNECTION_USB)
		{
			SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Failed to open a device. " <<
				"The LabJackDevice connection must be set to USB for the low-level driver used on Linux & Mac." <<
				std::endl <<
				"  Model: '" << m_model << "'.  Connection: '" << m_connection << "'.  Address: '" <<
				m_address << "'." << std::endl;
			result = false;
		}

		unsigned int deviceNumber = 1; // If no address is specified, grab the first device found of this model.
		if (m_address.length() > 0)
		{
			try
			{
				deviceNumber = std::stoi(m_address);
			}
			catch (int e)
			{
				SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Failed to open a device. " <<
					"The LabJackDevice address should be a string representation of an unsigned integer " <<
					"corresponding to the device number (or the empty string to get the first device), " <<
					"but the conversion from string to integer failed." << std::endl <<
					"  Model: '" << m_model << "'.  Connection: '" << m_connection << "'.  Address: '" <<
					m_address << "'." << std::endl;
				result = false;
			}
		}

		if (result)
		{
			const unsigned int dwReserved = 0; // Not used, set to 0.
			m_deviceHandle = LJUSB_OpenDevice(deviceNumber, dwReserved, m_model);
			if (m_deviceHandle == LABJACK_INVALID_HANDLE)
			{
				SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Failed to open a device." <<
					std::endl <<
					"  Model: '" << m_model << "'.  Connection: '" << m_connection << "'.  Address: '" <<
					m_address << "'." << std::endl <<
					"  labjackusb error code: " << errno << "." << std::endl;
				result = false;
			}
		}
	}

	/// Close communication with the hardware.
	/// \param reset true to cause a hardware reset & USB re-enumeration.  Otherwise the hardware's settings will be
	///		unchanged (i.e., it will continue timing, counting, and outputting).
	/// \return true.
	bool destroy(bool reset = false)
	{
		if (isValid())
		{
			if (reset)
			{
				std::array<BYTE, LabJack::MAXIMUM_BUFFER> sendBytes;
				sendBytes[1] = 0x99;  //Command byte, Reset
				sendBytes[2] = 2;  // Bit 1: Hard Reset.  Bit 0: Soft Reset.
				sendBytes[3] = 0x00;

				int sendBytesSize = 4;
				int readBytesSize = 4;

				sendBytes[0] = LabJack::normalChecksum8(sendBytes, sendBytesSize);

				const int sent = LJUSB_Write(m_deviceHandle, &(sendBytes[0]), sendBytesSize);

				bool sendResult = true;
				if (sent < sendBytesSize)
				{
					SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) <<
						"Failed to write reset command to a device." <<
						"  Model: '" << m_model << "'.  Connection: '" << m_connection << "'.  Address: '" <<
						m_address << "'." << std::endl << sendBytesSize << " bytes should have been sent, but only " <<
						sent << " were actually sent." << std::endl <<
						"  labjackusb error code: " << errno << "." << std::endl;
					sendResult = false;
				}

				if (sendResult)
				{
					std::array<BYTE, LabJack::MAXIMUM_BUFFER> readBytes;
					const int read = LJUSB_Read(m_deviceHandle, &(readBytes[0]), readBytesSize);
					if (read < readBytesSize)
					{
						SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Failed to read response of reset command." <<
							"  Model: '" << m_model << "'.  Connection: '" << m_connection << "'.  Address: '" <<
							m_address << "'." << std::endl <<
							readBytesSize << " bytes were expected, but only " << read << " were received." <<
							std::endl << "  labjackusb error code: " << errno << "." << std::endl;
					}
					else if (LabJack::normalChecksum8(readBytes, readBytesSize) != readBytes[0])
					{
						SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Failed to read response of reset command." <<
							"  Model: '" << m_model << "'.  Connection: '" << m_connection << "'.  Address: '" <<
							m_address << "'." << std::endl <<
							"The checksums are bad." << std::endl << "  labjackusb error code: " << errno << "." <<
							std::endl;
					}
					else if (readBytes[3] != 0)
					{
						SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Failed to read response of reset command." <<
							"  Model: '" << m_model << "'.  Connection: '" << m_connection << "'.  Address: '" <<
							m_address << "'." << std::endl <<
							"The device library returned an error code: " << static_cast<int>(readBytes[3]) << "." <<
							std::endl << "  labjackusb error code: " << errno << "." << std::endl;
					}
				}
			}

			LJUSB_CloseDevice(m_deviceHandle);
			m_deviceHandle = LABJACK_INVALID_HANDLE;
		}
		return true;
	}

	/// \return The LabJack SDK's handle wrapped by this Handle.
	LJ_HANDLE get() const
	{
		return m_deviceHandle;
	}

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	Handle(const Handle&) /*= delete*/;
	Handle& operator=(const Handle&) /*= delete*/;

	/// The exodriver device handle (or LABJACK_INVALID_HANDLE if not valid).
	LJ_HANDLE m_deviceHandle;
	/// The address used to open the device.  Can be the empty string if the first-found device was opened.
	std::string m_address;
	/// The model of the device.
	SurgSim::Device::LabJack::Model m_model;
	/// The connection to the device.
	SurgSim::Device::LabJack::Connection m_connection;
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
		thread(),
		deviceHandle(std::move(handle)),
		digitalInputChannels(device->getDigitalInputs()),
		digitalOutputChannels(device->getDigitalOutputs()),
		timerInputChannels(getTimerInputChannels(device->getTimers())),
		timerOutputChannels(getTimerOutputChannels(device->getTimers())),
		analogInputs(device->getAnalogInputs()),
		analogOutputChannels(device->getAnalogOutputs()),
		cachedOutputIndices(false),
		configured(false)
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
	/// The analog inputs.
	const std::unordered_map<int, LabJack::AnalogInputSettings> analogInputs;
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
	/// The DataGroup indices for the analog inputs.
	std::unordered_map<int, int> analogInputIndices;
	/// True if the output indices have been cached.
	bool cachedOutputIndices;
	/// Calibration constants.  The meaning of each entry is specific to the model (i.e., LabJack::Model).
	double calibration[40];
	/// True if the device has been successfully configured.
	bool configured;

private:
	/// Given all the timers, return just the ones that provide inputs.
	/// \param timers The timers.
	/// \return The timers that provide inputs.
	const std::unordered_set<int> getTimerInputChannels(const std::unordered_map<int,
		LabJack::TimerSettings>& timers) const
	{
		std::unordered_set<int> timersWithInputs;
		for (auto timer = timers.cbegin(); timer != timers.cend(); ++timer)
		{
			if ((timer->second.mode != LabJack::TIMERMODE_PWM_16BIT) &&
				(timer->second.mode != LabJack::TIMERMODE_PWM_8BIT) &&
				(timer->second.mode != LabJack::TIMERMODE_FREQUENCY_OUTPUT))
			{
				timersWithInputs.insert(timer->first);
			}
		}
		return timersWithInputs;
	}

	/// Given all the timers, return just the ones that take outputs.
	/// \param timers The timers.
	/// \return The timers that take outputs.
	const std::unordered_set<int> getTimerOutputChannels(const std::unordered_map<int,
		LabJack::TimerSettings>& timers) const
	{
		std::unordered_set<int> timersWithOutputs;
		for (auto timer = timers.cbegin(); timer != timers.cend(); ++timer)
		{
			if ((timer->second.mode == LabJack::TIMERMODE_PWM_16BIT) ||
				(timer->second.mode == LabJack::TIMERMODE_PWM_8BIT) ||
				(timer->second.mode == LabJack::TIMERMODE_RISING_EDGES_32BIT) ||
				(timer->second.mode == LabJack::TIMERMODE_FALLING_EDGES_32BIT) ||
				(timer->second.mode == LabJack::TIMERMODE_DUTY_CYCLE) ||
				(timer->second.mode == LabJack::TIMERMODE_FIRMWARE_COUNTER) ||
				(timer->second.mode == LabJack::TIMERMODE_FIRMWARE_COUNTER_DEBOUNCED) ||
				(timer->second.mode == LabJack::TIMERMODE_FREQUENCY_OUTPUT) ||
				(timer->second.mode == LabJack::TIMERMODE_QUADRATURE) ||
				(timer->second.mode == LabJack::TIMERMODE_RISING_EDGES_16BIT) ||
				(timer->second.mode == LabJack::TIMERMODE_FALLING_EDGES_16BIT) ||
				(timer->second.mode == LabJack::TIMERMODE_LINE_TO_LINE))
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
	SURGSIM_LOG_DEBUG(m_logger) << "Shared scaffold created.  labjackusb driver version: " <<
		LJUSB_GetLibraryVersion() << ".";
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
		const SurgSim::Device::LabJack::Model model = device->getModel();
		const SurgSim::Device::LabJack::Connection connection = device->getConnection();

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
		// If the device's model is SEARCH, iterate over the options.
		std::vector<LabJack::Model> modelsToSearch;
		if (device->getModel() == LabJack::MODEL_SEARCH)
		{
			SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << ": searching for models U3 and U6.";
			modelsToSearch.push_back(LabJack::MODEL_U6);
			modelsToSearch.push_back(LabJack::MODEL_U3);
		}
		else
		{
			modelsToSearch.push_back(device->getModel());
		}

		// If the device's connection is search, set it to USB since that is all we currently support for the
		// low-level driver.
		if (device->getConnection() == LabJack::CONNECTION_SEARCH)
		{
			SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << ": searching for connection. " <<
				"The low-level driver currently only supports USB as the medium.";
			device->setConnection(LabJack::CONNECTION_USB);
		}

		std::unique_ptr<Handle> handle;
		for (auto model = modelsToSearch.cbegin(); model != modelsToSearch.cend(); ++model)
		{
			device->setModel(*model);

			handle = std::unique_ptr<Handle>(new Handle(*model, device->getConnection(), address));
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
			std::unique_ptr<DeviceData> info(new DeviceData(device, std::move(handle)));

			// Cache the NamedData indices for the input DataGroup.
			const SurgSim::DataStructures::DataGroup& inputData = device->getInputData();

			for (auto input = info->digitalInputChannels.cbegin();
				input != info->digitalInputChannels.cend();
				++input)
			{
				info->digitalInputIndices[*input] =
					inputData.booleans().getIndex(SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX +
					std::to_string(*input));
				SURGSIM_ASSERT(info->digitalInputIndices[*input] >= 0) << "LabJackScaffold::DeviceData " <<
					"failed to get a valid NamedData index for the digital input for line " << *input <<
					".  Make sure that is a valid line number.";
			}

			for (auto timer = info->timerInputChannels.cbegin();
				timer != info->timerInputChannels.cend();
				++timer)
			{
				info->timerInputIndices[*timer] =
					inputData.scalars().getIndex(SurgSim::DataStructures::Names::TIMER_INPUT_PREFIX +
					std::to_string(*timer));
				SURGSIM_ASSERT(info->timerInputIndices[*timer] >= 0) << "LabJackScaffold::DeviceData " <<
					"failed to get a valid NamedData index for the timer for channel " << *timer <<
					".  Make sure that is a valid timer number.";
			}

			for (auto input = info->analogInputs.cbegin();
				input != info->analogInputs.cend(); ++input)
			{
				std::string name = SurgSim::DataStructures::Names::ANALOG_INPUT_PREFIX + std::to_string(input->first);
				info->analogInputIndices[input->first] = inputData.scalars().getIndex(name);
				SURGSIM_ASSERT(info->analogInputIndices[input->first] >= 0) <<
					"LabJackScaffold::DeviceData failed to get a valid NamedData index for the " <<
					"analog input for channel " << input->first << ".  Make sure that is a valid line number.  " <<
					"Expected an entry named " << name << ".";
			}

			std::unique_ptr<LabJackThread> thread(new LabJackThread(this, info.get()));
			result = info->configured;
			if (result)
			{
				thread->setRate(device->getMaximumUpdateRate());
				thread->start();
				info.get()->thread = std::move(thread);
				m_state->activeDeviceList.emplace_back(std::move(info));
			}
			else
			{
				info->deviceHandle->destroy();
			}
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
				matching->get()->deviceHandle->destroy(matching->get()->deviceObject->getResetOnDestruct());
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
				initialOutputData.booleans().getIndex(SurgSim::DataStructures::Names::DIGITAL_OUTPUT_PREFIX +
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

	if (!updateDevice(info))
	{
		return false;
	}
	info->deviceObject->pushInput();

	return true;
}

bool LabJackScaffold::updateDevice(LabJackScaffold::DeviceData* info)
{
	const LJ_HANDLE rawHandle = info->deviceHandle->get();
	const SurgSim::DataStructures::DataGroup& outputData = info->deviceObject->getOutputData();
	const LabJackDevice* device = info->deviceObject;
	bool result = true;

	// Use one Feedback command for all the inputs and outputs.
	// According to the Users Guide, the Feedback command has a max size of 64 bytes. Currently only a single command is
	// used, and we try to add all the input/output information. To be safe the std::array::at function is used for any
	// access to a non-constant index, and will throw an out_of_range exception if the index is too large.
	std::array<BYTE, LabJack::MAXIMUM_BUFFER> sendBytes;
	sendBytes[1] = 0xF8;  //Command byte, Feedback
	sendBytes[3] = 0x00;  //Extended command number

	sendBytes[6] = 0;     //Echo

	int sendBytesSize = 7; // the first IOType goes here
	int readBytesSize = 9; // Reading after a Feedback command provides 9+ bytes.

	// Read digital inputs
	const std::unordered_set<int>& digitalInputChannels = info->digitalInputChannels;
	for (auto input = digitalInputChannels.cbegin(); input != digitalInputChannels.cend(); ++input)
	{
		sendBytes.at(sendBytesSize++) = 10;  //IOType, BitStateRead
		sendBytes.at(sendBytesSize++) = *input;  //Bits 0-4: IO Number
		++readBytesSize;
	}

	// Write digital outputs
	const std::unordered_set<int>& digitalOutputChannels = info->digitalOutputChannels;
	for (auto output = digitalOutputChannels.cbegin(); output != digitalOutputChannels.cend(); ++output)
	{
		if (info->digitalOutputIndices.count(*output) > 0)
		{
			const int index = info->digitalOutputIndices[*output];
			SURGSIM_ASSERT(index >= 0) << "LabJackScaffold: A LabJackDevice was configured with line " << *output <<
				" set to digital output, but the scaffold does not know the correct index into the NamedData. " <<
				" Make sure there is an entry in the booleans with the correct string key.";

			bool value;
			if (outputData.booleans().get(index, &value))
			{
				const BYTE valueAsByte = (value ? 1 : 0);
				sendBytes.at(sendBytesSize++) = 11;  //IOType, BitStateWrite
				sendBytes.at(sendBytesSize++) = (*output) | ((valueAsByte & 0xF) << 7);  //Bits 0-4: IO Number,
																						//Bit 7: State
			}
		}
	}

	// Write to timers (e.g., resetting firmware counters).
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
					const int valueAsInt = value + 0.5; // value is non-negative.  The conversion will truncate.
					sendBytes.at(sendBytesSize++) = 42 + 2 * (*timer);  //IOType, Timer#
					sendBytes.at(sendBytesSize++) = 1;  //Bit 0: UpdateReset
					sendBytes.at(sendBytesSize++) = valueAsInt & 255;  //LSB
					sendBytes.at(sendBytesSize++) = (valueAsInt & 65280) / 256;  //MSB
					readBytesSize += 4;
				}
			}
		}
	}

	// Read from timers.
	const std::unordered_set<int>& timerInputChannels = info->timerInputChannels;
	for (auto timer = timerInputChannels.cbegin(); timer != timerInputChannels.cend(); ++timer)
	{
		// If we write to a timer, the response to that write will have the timer value so we don't need to request it.
		if (timerOutputChannels.count(*timer) == 0)
		{
			sendBytes.at(sendBytesSize++) = 42 + 2 * (*timer);  //IOType, Timer#
			sendBytes.at(sendBytesSize++) = 0;  //Bit 0: UpdateReset
			sendBytes.at(sendBytesSize++) = 0;  //LSB
			sendBytes.at(sendBytesSize++) = 0;  //MSB
			readBytesSize += 4;
		}
	}

	// Request the values of analog inputs.
	auto const& analogInputs = info->analogInputs;
	for (auto input = analogInputs.cbegin(); input != analogInputs.cend(); ++input)
	{
		if (device->getModel() == LabJack::MODEL_U3)
		{
			const BYTE longSettling = (device->getAnalogInputSettling() > 0 ? (1 << 6) : 0);
			const BYTE quickSample = (device->getAnalogInputResolution() > 0 ? (1 << 7) : 0);
			// Third byte is the negative channel for differential input, or 31 for single-ended input.
			const BYTE negativeChannel = input->second.negativeChannel.hasValue() ?
				input->second.negativeChannel.getValue() : 31;
			sendBytes.at(sendBytesSize++) = 1;
			sendBytes.at(sendBytesSize++) = input->first | longSettling | quickSample;
			sendBytes.at(sendBytesSize++) = negativeChannel;
			readBytesSize += 2;
		}
		else
		{
			// The U6 can only do consecutive channels for a differential read, so the actual channel is not sent.
			const BYTE differential = input->second.negativeChannel.hasValue() ? 1 << 7 : 0;
			sendBytes.at(sendBytesSize++) = 3;
			sendBytes.at(sendBytesSize++) = input->first;
			sendBytes.at(sendBytesSize++) = device->getAnalogInputResolution() | (getGain(input->second.range) << 4);
			sendBytes.at(sendBytesSize++) = device->getAnalogInputSettling() | differential;
			readBytesSize += 5;
		}
	}

	// Set analog outputs.
	const std::unordered_set<int>& analogOutputs = info->analogOutputChannels;
	for (auto output = analogOutputs.cbegin(); output != analogOutputs.cend(); ++output)
	{
		if (info->analogOutputIndices.count(*output) > 0)
		{
			const int index = info->analogOutputIndices[*output];
			if (index >= 0)
			{
				double value;
				if (outputData.scalars().get(index, &value))
				{
					const BYTE dacConfigCode = *output + 38;
					sendBytes.at(sendBytesSize++) = dacConfigCode;

					int bytes;
					if (device->getModel() == LabJack::MODEL_U3)
					{
						const int calibrationIndex = *output * 2 + 4;
						bytes = value * info->calibration[calibrationIndex] * 256 +
							info->calibration[calibrationIndex + 1] * 256;
					}
					else
					{
						const int calibrationIndex = *output * 2 + 16;
						bytes = value * info->calibration[calibrationIndex] + info->calibration[calibrationIndex + 1];
					}
					bytes = std::min(bytes, 65535);
					bytes = std::max(bytes, 0);
					sendBytes.at(sendBytesSize++) = bytes & 255;
					sendBytes.at(sendBytesSize++) = bytes / 256;
				}
			}
		}
	}

	// Write the Feedback command.
	// Must send an even number of bytes.
	if (sendBytesSize % 2 == 1)
	{
		sendBytes.at(sendBytesSize++) = 0;
	}

	// Must read an even number of bytes
	if (readBytesSize % 2 == 1)
	{
		++readBytesSize;
	}

	// According to the Users Guide, the Feedback command has a max Response/Read size of 64 bytes.
	// Most IOTypes Read no more bytes than they Write, including the digital input/output and timers currently
	// supported, but some IOTypes do Read more than they Write. Thus, this assertion should never be triggered, and
	// to make sure the access indices into the buffer are being correctly calculated, the std::array::at function is
	// used for any access to a non-constant index, and will throw an out_of_range exception if the index is too large.
	if (readBytesSize > LabJack::MAXIMUM_BUFFER)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Failed to update device named '" << device->getName() <<
		"'. The low-level LabJack communication function 'Feedback' has a max Response size of " <<
		std::to_string(LabJack::MAXIMUM_BUFFER) << " bytes. " <<
		"The current configuration and output DataGroup would cause a size of " << readBytesSize << " bytes." <<
		std::endl;
		result = false;
	}

	if (result)
	{
		const BYTE sendCommandBytes = 6;
		const BYTE dataWords = (sendBytesSize - sendCommandBytes) / 2;
		sendBytes[2] = dataWords;
		LabJack::extendedChecksum(&sendBytes, sendBytesSize);

		const int sent = LJUSB_Write(rawHandle, &(sendBytes[0]), sendBytesSize);

		if (sent < sendBytesSize)
		{
			SURGSIM_LOG_SEVERE(m_logger) <<
				"Failed to write feedback command to update a device named '" <<
				device->getName() << "'.  " << sendBytesSize << " bytes should have been sent, but only " <<
				sent << " were actually sent." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
	}

	const BYTE readCommandBytes = 6;
	std::array<BYTE, LabJack::MAXIMUM_BUFFER> readBytes; // Feedback commands respond with a max of 64 bytes

	// Read the response for the feedback command.
	if (result)
	{
		const std::string errorText = "feedback command to update";
		result = readAndCheck(rawHandle, &readBytes, readBytesSize, device->getName(), sendBytes, errorText, m_logger);

		const BYTE dataWords = (readBytesSize - readCommandBytes) / 2;
		if (readBytes[2] != dataWords)
		{
			SURGSIM_LOG_SEVERE(m_logger) <<
				"Failed to read response of " << errorText << " a device named '" <<	device->getName() <<
				"'.  The number of words in the response is wrong.  Expected: " << static_cast<int>(dataWords) <<
				".  Received: " << static_cast<int>(readBytes[2]) << "." << std::endl << "  labjackusb error code: " <<
				errno << "." << std::endl;
			result = false;
		}
	}

	SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();
	if (result)
	{
		const int errorBytes = 2;
		const int echoBytes = 1;
		int currentByte = readCommandBytes + errorBytes + echoBytes;

		// Digital inputs.
		for (auto input = digitalInputChannels.cbegin(); input != digitalInputChannels.cend(); ++input)
		{
			const bool value = (readBytes.at(currentByte++) > 0 ? true : false);
			inputData.booleans().set(info->digitalInputIndices[*input], value);
		}

		// Read from the timers in the same order as above.
		for (auto timer = timerOutputChannels.cbegin(); timer != timerOutputChannels.cend(); ++timer)
		{
			const int count = 4;
			const uint32_t value = LabJack::uint32FromChars(readBytes, currentByte, count);
			inputData.scalars().set(info->timerInputIndices[*timer], value);
			currentByte += 4;
		}

		for (auto timer = timerInputChannels.cbegin(); timer != timerInputChannels.cend(); ++timer)
		{
			if (timerOutputChannels.count(*timer) == 0)
			{
				const int count = 4;
				const uint32_t value = LabJack::uint32FromChars(readBytes, currentByte, count);
				currentByte += 4;
				// Interpret the value as signed.
				inputData.scalars().set(info->timerInputIndices[*timer], value);
			}
		}

		for (auto input = analogInputs.cbegin(); input != analogInputs.cend(); ++input)
		{
			if (device->getModel() == LabJack::MODEL_U3)
			{
				const int count = 2;
				const uint16_t value = LabJack::uint16FromChars(readBytes, currentByte, count);
				currentByte += 2;
				const double volts = info->calibration[2] * static_cast<double>(value) + info->calibration[3];
				inputData.scalars().set(info->analogInputIndices[input->first], volts);
			}
			else
			{
				const int count = 3;
				const uint32_t value = LabJack::uint32FromChars(readBytes, currentByte, count);
				double bits = value;
				bits /= 256.0;

				const int gain = readBytes.at(currentByte + 3) / 16;
				int index = gain * 2 + 8;

				const int resolution = readBytes.at(currentByte + 3) & 15;
				if (resolution > 8)
				{
					index += 24;
				}

				currentByte += 5;

				double volts;
				const double center = info->calibration[index + 1];
				if (bits < center)
				{
					volts = (center - bits) * info->calibration[index];
				}
				else
				{
					volts = (bits - center) * info->calibration[index - 8];
				}
				inputData.scalars().set(info->analogInputIndices[input->first], volts);
			}
		}
	}
	else
	{
		// Failure communicating with the LabJack
		inputData.resetAll();
	}

	return true; // Returning false ends the thread.
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
		builder.addBoolean(SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX + std::to_string(i));
	}

	const int maxTimerInputs = 6; // The UE9 can have 6 timers.
	for (int i = 0; i < maxTimerInputs; ++i)
	{
		builder.addScalar(SurgSim::DataStructures::Names::TIMER_INPUT_PREFIX + std::to_string(i));
	}

	const int maxAnalogInputs = 16; // The U3 can have 16 analog inputs.
	for (int i = 0; i < maxAnalogInputs; ++i)
	{
		builder.addScalar(SurgSim::DataStructures::Names::ANALOG_INPUT_PREFIX + std::to_string(i));
	}
	return builder.createData();
}

std::shared_ptr<LabJackScaffold> LabJackScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<LabJackScaffold> sharedInstance;
	return sharedInstance.get();
}

void LabJackScaffold::configureDevice(DeviceData* deviceData)
{
	deviceData->configured =
		configureClockAndTimers(deviceData) && configureDigital(deviceData) && configureAnalog(deviceData);
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

	// One-time configuration of counters and timers.
	const std::unordered_map<int, LabJack::TimerSettings>& timers = device->getTimers();

	for (auto timer : timers)
	{
		SURGSIM_LOG_IF(timer.first >= static_cast<int>(timers.size()), m_logger, SEVERE) <<
			"Error configuring enabled timers for a device named '" << device->getName() <<
			"', with number of timers: " << timers.size() << "." << std::endl <<
			"  Timers must be enabled consecutively, starting with #0." << std::endl <<
			"  With the currently enabled number of timers, the highest allowable timer is #" <<
			timers.size() - 1 << ", but one of the enabled timers is #" << timer.first << "." << std::endl;
	}

	const BYTE numberOfTimers = timers.size();
	const BYTE counterEnable = 0; // Counters are not currently supported.
	const BYTE pinOffset = device->getTimerCounterPinOffset();

	// First, configure the number of timers and counters
	std::array<BYTE, LabJack::MAXIMUM_BUFFER> sendBytes;
	sendBytes[1] = 0xF8;  //Command byte, ConfigIO
	sendBytes[2] = 0x05;  //Number of data words
	sendBytes[3] = 0x0B;  //Extended command number

	sendBytes[6] = 1;  //Writemask : Setting writemask for timerCounterConfig (bit 0)

	sendBytes[7] = numberOfTimers;      //NumberTimersEnabled
	sendBytes[8] = counterEnable;  //CounterEnable: Bit 0 is Counter 0, Bit 1 is Counter 1
	sendBytes[9] = pinOffset;  //TimerCounterPinOffset

	int sendBytesSize = 16; //ConfigIO command sends 16 bytes
	for (int i = 10; i < sendBytesSize; ++i)
	{
		sendBytes[i] = 0;  //Reserved
	}
	LabJack::extendedChecksum(&sendBytes, 16);

	int sent = LJUSB_Write(rawHandle, &(sendBytes[0]), sendBytesSize);

	bool result = true;

	if (sent < sendBytesSize)
	{
		SURGSIM_LOG_SEVERE(m_logger) <<
			"Failed to write timer/counter configuration information for a device named '" << device->getName() <<
			"'.  " << sendBytesSize << " bytes should have been sent, but only " << sent <<
			" were actually sent." << std::endl << "  labjackusb error code: " << errno << "." << std::endl;
		result = false;
	}

	if (result)
	{
		const int readBytesSize = 16; // timerCounterConfig replies with 16 bytes.
		std::array<BYTE, LabJack::MAXIMUM_BUFFER> readBytes;

		const std::string errorText = "timer/counter configuration for";
		result = readAndCheck(rawHandle, &readBytes, readBytesSize, device->getName(), sendBytes, errorText, m_logger);

		if (readBytes[2] != sendBytes[2])
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Failed to read response of " << errorText << " a device named '" <<
				device->getName() << "'.  The command bytes are wrong.  Expected: " << static_cast<int>(sendBytes[1]) <<
				", " << static_cast<int>(sendBytes[2]) << ", " << static_cast<int>(sendBytes[3]) << ".  Received: " <<
				static_cast<int>(readBytes[1]) << ", " << static_cast<int>(readBytes[2]) << ", " <<
				static_cast<int>(readBytes[3]) << "." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
	}
	return result;
}

bool LabJackScaffold::configureClock(DeviceData* deviceData)
{
	LabJackDevice* device = deviceData->deviceObject;
	LJ_HANDLE rawHandle = deviceData->deviceHandle->get();

	LabJack::TimerBase base = device->getTimerBase();
	if (base == LabJack::TimerBase::TIMERBASE_DEFAULT)
	{
		LabJackDefaults defaults;
		base = defaults.timerBase[device->getModel()];
	}

	// Second, set the clock base and divisor

	// The low-level settings for the timer base are 20 less than the high-level values for the U3 and U6.
	// The low- and high-level settings are the same for the UE9.
	const int timerBaseOffset = 20;
	BYTE timerBase = device->getTimerBase();
	// If the TimerBase is above 20, it was set for a high-level value, convert to the equivalent value for
	// the low-level driver.
	if (timerBase > timerBaseOffset)
	{
		timerBase -= timerBaseOffset;
	}
	const BYTE divisor = device->getTimerClockDivisor();

	// Write the clock configuration.
	std::array<BYTE, LabJack::MAXIMUM_BUFFER> sendBytes;
	sendBytes[1] = 0xF8;  //Command byte, ConfigTimerClock
	sendBytes[2] = 0x02;  //Number of data words
	sendBytes[3] = 0x0A;  //Extended command number

	sendBytes[6] = 0;  //Reserved
	sendBytes[7] = 0;  //Reserved

	//TimerClockConfig : Configuring the clock (bit 7) and setting the TimerClockBase (bits 0-2)
	const BYTE configureClockCode = 1 << 7;
	sendBytes[8] = timerBase | configureClockCode;
	sendBytes[9] = divisor;  //TimerClockDivisor

	int sendBytesSize = 10; // ConfigTimerClock sends 10 bytes
	LabJack::extendedChecksum(&sendBytes, sendBytesSize);

	int sent = LJUSB_Write(rawHandle, &(sendBytes[0]), sendBytesSize);

	bool result = true;

	if (sent < sendBytesSize)
	{
		SURGSIM_LOG_SEVERE(m_logger) <<
			"Failed to write clock configuration information for a device named '" <<
			device->getName() << "'.  " << sendBytesSize << " bytes should have been sent, but only " <<
			sent << " were actually sent." << std::endl << "  labjackusb error code: " << errno << "." <<
			std::endl;
		result = false;
	}

	if (result)
	{
		// Read the response for the clock configuration.
		const int readBytesSize = 10; // ConfigTimerClock replies with 10 bytes.
		std::array<BYTE, LabJack::MAXIMUM_BUFFER> readBytes;

		const std::string errorText = "clock configuration for";
		result = readAndCheck(rawHandle, &readBytes, readBytesSize, device->getName(), sendBytes, errorText, m_logger);

		if (readBytes[2] != sendBytes[2])
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Failed to read response of " << errorText << " a device named '" <<
				device->getName() << "'.  The command bytes are wrong.  Expected: " << static_cast<int>(sendBytes[1]) <<
				", " << static_cast<int>(sendBytes[2]) << ", " << static_cast<int>(sendBytes[3]) << ".  Received: " <<
				static_cast<int>(readBytes[1]) << ", " << static_cast<int>(readBytes[2]) << ", " <<
				static_cast<int>(readBytes[3]) << "." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
	}
	return result;
}

bool LabJackScaffold::configureTimers(DeviceData* deviceData)
{
	LabJackDevice* device = deviceData->deviceObject;
	LJ_HANDLE rawHandle = deviceData->deviceHandle->get();

	bool result = true;

	// Configure each timer's mode via a Feedback command.
	std::array<BYTE, LabJack::MAXIMUM_BUFFER> sendBytes;
	sendBytes[1] = 0xF8;  //Command byte, Feedback
	sendBytes[3] = 0x00;  //Extended command number

	sendBytes[6] = 0;     //Echo
	int sendBytesSize = 7; // the first IOType goes here

	int readBytesSize = 9; // Reading after a Feedback command provides 9+ bytes.
	const std::unordered_map<int, LabJack::TimerSettings> timers = device->getTimers();
	for (auto timer = timers.cbegin(); timer != timers.cend(); ++timer)
	{
		const int minimumTimer = 0;
		const int maximumTimer = 3;
		if ((timer->first < minimumTimer) || (timer->first > maximumTimer))
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Failed to configure a timer for a device named '" <<
				device->getName() << "'. Timer number (the key in the argument to LabJackDevice::setTimers) " <<
				timer->first << " is outside of the valid range " << minimumTimer << "-" << maximumTimer <<
				"." << std::endl;
			result = false;
		}

		if (result)
		{
			const BYTE timerConfigCode = timer->first * 2 + 43; // The timer configs are 43, 45, 47, and 49.
			sendBytes.at(sendBytesSize++) = timerConfigCode;  //IOType, Timer#Config (e.g., Timer0Config)
			sendBytes.at(sendBytesSize++) = timer->second.mode;  //TimerMode

			const int initialValue = timer->second.initialValue.hasValue() ? timer->second.initialValue.getValue() : 0;
			sendBytes.at(sendBytesSize++) = static_cast<BYTE>(initialValue & 0xFF);  //Value LSB
			sendBytes.at(sendBytesSize++) = static_cast<BYTE>((initialValue >> 8) & 0xFF);  //Value MSB
		}
	}

	if (result)
	{
		// Write the timer Feedback command.
		// Must send an even number of bytes.
		if (sendBytesSize % 2 == 1)
		{
			sendBytes.at(sendBytesSize++) = 0;
		}
		const BYTE sendCommandBytes = 6;
		const BYTE dataWords = (sendBytesSize - sendCommandBytes) / 2;
		sendBytes[2] = dataWords;
		LabJack::extendedChecksum(&sendBytes, sendBytesSize);

		int sent = LJUSB_Write(rawHandle, &(sendBytes[0]), sendBytesSize);

		if (sent < sendBytesSize)
		{
			SURGSIM_LOG_SEVERE(m_logger) <<
				"Failed to write timer mode feedback command for a device named '" <<
				device->getName() << "'.  " << sendBytesSize << " bytes should have been sent, but only " <<
				sent << " were actually sent." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
	}

	if (result)
	{
		// Read the response for the timer feedback command.
		// Must read an even number of bytes
		if (readBytesSize % 2 == 1)
		{
			++readBytesSize;
		}

		std::array<BYTE, LabJack::MAXIMUM_BUFFER> readBytes;
		const std::string errorText = "timer mode feedback command for";
		result = readAndCheck(rawHandle, &readBytes, readBytesSize, device->getName(), sendBytes, errorText, m_logger);

		const BYTE readCommandBytes = 6;
		const BYTE dataWords = (readBytesSize - readCommandBytes) / 2;
		if (readBytes[2] != dataWords)
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Failed to read response of " << errorText << " a device named '" <<
				device->getName() << "'.  The number of words in the response is wrong.  Expected: " <<
				static_cast<int>(dataWords) << ".  Received: " << static_cast<int>(readBytes[2]) << "." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
	}
	return result;
}

bool LabJackScaffold::configureAnalog(DeviceData* deviceData)
{
	LabJackDevice* device = deviceData->deviceObject;
	LJ_HANDLE rawHandle = deviceData->deviceHandle->get();

	bool result = true;

	const std::unordered_set<int>& analogOutputs = deviceData->analogOutputChannels;
	for (auto output = analogOutputs.cbegin(); output != analogOutputs.cend(); ++output)
	{
		const int minimumDac = 0;
		const int maximumDac = 1;
		if ((*output < minimumDac) || (*output > maximumDac))
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Failed to configure an analog output (DAC) for a device named '" <<
				device->getName() << "'. DAC number " << *output << " is outside of the valid range " <<
				minimumDac << "-" << maximumDac << "." << std::endl;
			result = false;
		}
	}

	if (device->getModel() == LabJack::MODEL_U6)
	{
		auto const& analogInputs = deviceData->analogInputs;
		for (auto input = analogInputs.cbegin(); input != analogInputs.cend(); ++input)
		{
			if ((input->second.negativeChannel.hasValue()) &&
				(input->second.negativeChannel.getValue() != input->first + 1))
			{
				SURGSIM_LOG_SEVERE(m_logger) <<
					"Failed to configure a differential analog input for a device named '" <<
					device->getName() << "'. For a model U6(-PRO), with the low-level driver, "<<
					"the negative channel number must be one greater than the positive channel number, " <<
					"but positive channel " << input->first << " is paired with negative channel " <<
					input->second.negativeChannel.getValue() << ".";
				result = false;
			}
		}
	}

	LabJackParameters parameters;
	const int blocks = parameters.configBlocks[device->getModel()];
	std::array<BYTE, LabJack::MAXIMUM_BUFFER> sendBytes;
	sendBytes[1] = 0xF8; //command byte
	sendBytes[2] = 0x01; //number of data words
	sendBytes[3] = parameters.calibrationCommand[device->getModel()];
	sendBytes[6] = 0;
	const int sendBytesSize = 8;
	const int readBytesSize = parameters.calibrationReadBytes[device->getModel()];

	for (int i = 0; i < blocks; ++i)
	{
		if (result)
		{
			sendBytes[7] = i;
			LabJack::extendedChecksum(&sendBytes, sendBytesSize);

			const int sent = LJUSB_Write(rawHandle, &(sendBytes[0]), sendBytesSize);
			if (sent < sendBytesSize)
			{
				SURGSIM_LOG_SEVERE(m_logger) <<
					"Failed to write a feedback command to read the calibration data for a device named '" <<
					device->getName() << "'.  " << sendBytesSize << " bytes should have been sent, but only " <<
					sent << " were actually sent." << std::endl <<
					"  labjackusb error code: " << errno << "." << std::endl;
				result = false;
			}
		}

		if (result)
		{
			std::array<BYTE, LabJack::MAXIMUM_BUFFER> readBytes;
			const std::string errorText = "feedback command to read the calibration data for";
			result = readAndCheck(rawHandle, &readBytes, readBytesSize, device->getName(), sendBytes, errorText,
				m_logger);

			if (readBytes[2] != parameters.calibrationThirdByte[device->getModel()])
			{
				SURGSIM_LOG_SEVERE(m_logger) << "Failed to read response of " << errorText << " a device named '" <<
					device->getName() << "'.  The command bytes are wrong.  Expected byte 2: " <<
					static_cast<int>(parameters.calibrationThirdByte[device->getModel()]) << ".  Received: " <<
					static_cast<int>(readBytes[2]) << "." << std::endl << "  labjackusb error code: " << errno << "." <<
					std::endl;
				result = false;
			}

			if (result)
			{
				const int offset = i * 4;
				deviceData->calibration[offset] = LabJack::doubleFromChars(readBytes, 8);
				deviceData->calibration[offset + 1] = LabJack::doubleFromChars(readBytes, 16);
				deviceData->calibration[offset + 2] = LabJack::doubleFromChars(readBytes, 24);
				deviceData->calibration[offset + 3] = LabJack::doubleFromChars(readBytes, 32);
			}
		}
	}

	return result;
}

bool LabJackScaffold::configureDigital(DeviceData* deviceData)
{
	LabJackDevice* device = deviceData->deviceObject;
	LJ_HANDLE rawHandle = deviceData->deviceHandle->get();

	bool result = true;

	const std::unordered_set<int>& digitalInputChannels = deviceData->digitalInputChannels;
	const std::unordered_set<int>& digitalOutputChannels = deviceData->digitalOutputChannels;
	if ((digitalInputChannels.size() > 0) || (digitalOutputChannels.size() > 0))
	{
		// Configure each digital line's direction via a Feedback command.  The output lines will automatically be
		// forced high when we write their states in updateDevice, but we must explicitly set the direction on
		// input lines so we just do both here.
		std::array<BYTE, LabJack::MAXIMUM_BUFFER> sendBytes;
		sendBytes[1] = 0xF8;  //Command byte, Feedback
		sendBytes[3] = 0x00;  //Extended command number

		sendBytes[6] = 0;     //Echo
		int sendBytesSize = 7; // the first IOType goes here

		int readBytesSize = 9; // Reading after a Feedback command provides 9+ bytes.
		for (auto input = digitalInputChannels.cbegin(); input != digitalInputChannels.cend(); ++input)
		{
			sendBytes.at(sendBytesSize++) = 13;  //IOType, BitDirWrite
			sendBytes.at(sendBytesSize++) = *input;  //Bits 0-4: IO Number, Bit 7: Direction (1=output, 0=input)
		}
		for (auto output = digitalOutputChannels.cbegin(); output != digitalOutputChannels.cend(); ++output)
		{
			const BYTE direction = 1 << 7;
			sendBytes.at(sendBytesSize++) = 13;  //IOType, BitDirWrite
			sendBytes.at(sendBytesSize++) = *output | direction;  //Bits 0-4: IO Number,
																	// Bit 7: Direction (1=output, 0=input)
		}

		// Write the digital input/output Feedback command.
		// Must send an even number of bytes.
		if (sendBytesSize % 2 == 1)
		{
			sendBytes.at(sendBytesSize++) = 0;
		}
		const BYTE sendCommandBytes = 6;
		const BYTE dataWords = (sendBytesSize - sendCommandBytes) / 2;
		sendBytes[2] = dataWords;
		LabJack::extendedChecksum(&sendBytes, sendBytesSize);

		int sent = LJUSB_Write(rawHandle, &(sendBytes[0]), sendBytesSize);

		if (sent < sendBytesSize)
		{
			SURGSIM_LOG_SEVERE(m_logger) <<
				"Failed to write digital line direction feedback command for a device named '" <<
				device->getName() << "'.  " << sendBytesSize << " bytes should have been sent, but only " <<
				sent << " were actually sent." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}

		if (result)
		{
			// Read the response for the digital input/output feedback command.
			// Must read an even number of bytes
			if (readBytesSize % 2 == 1)
			{
				++readBytesSize;
			}

			std::array<BYTE, LabJack::MAXIMUM_BUFFER> readBytes;
			const std::string errorText = "digital line direction configuration for";
			result = readAndCheck(rawHandle, &readBytes, readBytesSize, device->getName(), sendBytes, errorText,
				m_logger);

			const BYTE readCommandBytes = 6;
			const BYTE dataWords = (readBytesSize - readCommandBytes) / 2;
			if (readBytes[2] != dataWords)
			{
				SURGSIM_LOG_SEVERE(m_logger) << "Failed to read response of " << errorText << " a device named '" <<
					device->getName() << "'.  The number of words in the response is wrong.  Expected: " <<
					static_cast<int>(dataWords) << ".  Received: " << static_cast<int>(readBytes[2]) << "." <<
					std::endl << "  labjackusb error code: " << errno << "." << std::endl;
				result = false;
			}
		}
	}
	return result;
}

std::shared_ptr<SurgSim::Framework::Logger> LabJackScaffold::getLogger() const
{
	return m_logger;
}



};  // namespace Device
};  // namespace SurgSim

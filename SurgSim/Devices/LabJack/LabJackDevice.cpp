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

#include "SurgSim/Devices/LabJack/LabJackDevice.h"

#include "SurgSim/Devices/LabJack/LabJackScaffold.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::LabJackDevice, LabJackDevice);

LabJackDevice::LabJackDevice(const std::string& uniqueName) :
	SurgSim::Input::CommonDevice(uniqueName, LabJackScaffold::buildDeviceInputData()),
	m_model(LabJack::MODEL_SEARCH),
	m_connection(LabJack::CONNECTION_SEARCH),
	m_address(""),
	m_reset(false),
	m_timerBase(LabJack::TIMERBASE_DEFAULT),
	m_timerClockDivisor(1),
	m_timerCounterPinOffset(0),
	m_threadRate(1000.0),
	m_analogInputResolution(0),
	m_analogInputSettling(0)
{
}

LabJackDevice::~LabJackDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}

bool LabJackDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized()) << "LabJackDevice already initialized.";

	std::shared_ptr<LabJackScaffold> scaffold = LabJackScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold != nullptr) << "LabJackDevice failed to get a LabJackScaffold.";

	auto logger = Framework::Logger::getLogger("Devices/LabJack");
	SURGSIM_LOG_IF((getDigitalOutputs().size() > 0) && !hasOutputProducer(), logger, WARNING) <<
		"LabJackDevice named " << getName() <<
		" has digital output channels but no output producer to provide the output data. Call setOutputProducer.";

	SURGSIM_LOG_IF((getAnalogOutputs().size() > 0) && !hasOutputProducer(), logger, WARNING) <<
		"LabJackDevice named " << getName() <<
		" has analog output channels but no output producer to provide the output data. Call setOutputProducer.";

	bool registered = false;
	// registerDevice will set this object's type and/or connection, if they are currently set to SEARCH.
	if (scaffold->registerDevice(this))
	{
		m_scaffold = std::move(scaffold);
		registered = true;
	}
	return registered;
}

bool LabJackDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << "LabJackDevice has not been initialized before finalize.";
	const bool ok = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();
	return ok;
}

bool LabJackDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}

void LabJackDevice::setModel(LabJack::Model model)
{
	SURGSIM_ASSERT(!isInitialized()) << "LabJackDevice's model cannot be set after it is initialized.";
	m_model = model;
}

LabJack::Model LabJackDevice::getModel() const
{
	return m_model;
}

void LabJackDevice::setConnection(LabJack::Connection connection)
{
	SURGSIM_ASSERT(!isInitialized()) << "LabJackDevice's connection cannot be set after it is initialized.";
	m_connection = connection;
}

LabJack::Connection LabJackDevice::getConnection() const
{
	return m_connection;
}

void LabJackDevice::setAddress(std::string address)
{
	SURGSIM_ASSERT(!isInitialized()) << "LabJackDevice's address cannot be set after it is initialized.";
	m_address = address;
}

const std::string& LabJackDevice::getAddress() const
{
	return m_address;
}

void LabJackDevice::setResetOnDestruct(bool reset)
{
	m_reset = reset;
}

bool LabJackDevice::getResetOnDestruct() const
{
	return m_reset;
}

void LabJackDevice::enableDigitalInput(int channel)
{
	SURGSIM_ASSERT(!isInitialized()) << "Digital input cannot be enabled for a LabJackDevice after it is initialized.";
	m_digitalInputChannels.insert(channel);
}

void LabJackDevice::setDigitalInputs(const std::unordered_set<int>& digitalInputChannels)
{
	SURGSIM_ASSERT(!isInitialized()) << "Digital inputs cannot be enabled for a LabJackDevice after it is initialized.";
	m_digitalInputChannels = digitalInputChannels;
}

const std::unordered_set<int>& LabJackDevice::getDigitalInputs() const
{
	return m_digitalInputChannels;
}

void LabJackDevice::enableDigitalOutput(int channel)
{
	SURGSIM_ASSERT(!isInitialized()) << "Digital output cannot be enabled for a LabJackDevice after it is initialized.";
	m_digitalOutputChannels.insert(channel);
}

void LabJackDevice::setDigitalOutputs(const std::unordered_set<int>& digitalOutputChannels)
{
	SURGSIM_ASSERT(!isInitialized()) <<
		"Digital outputs cannot be enabled for a LabJackDevice after it is initialized.";
	m_digitalOutputChannels = digitalOutputChannels;
}

const std::unordered_set<int>& LabJackDevice::getDigitalOutputs() const
{
	return m_digitalOutputChannels;
}

void LabJackDevice::setTimerBase(LabJack::TimerBase base)
{
	SURGSIM_ASSERT(!isInitialized()) << "LabJackDevice's timer base cannot be set after it is initialized.";
	m_timerBase = base;
}

LabJack::TimerBase LabJackDevice::getTimerBase() const
{
	return m_timerBase;
}

void LabJackDevice::setTimerClockDivisor(int divisor)
{
	SURGSIM_ASSERT(!isInitialized()) << "LabJackDevice's timer clock divisor cannot be set after it is initialized.";
	m_timerClockDivisor = divisor;
}

int LabJackDevice::getTimerClockDivisor() const
{
	return m_timerClockDivisor;
}

void LabJackDevice::setTimerCounterPinOffset(int offset)
{
	SURGSIM_ASSERT(!isInitialized()) <<
		"LabJackDevice's timer/counter pin offset cannot be set after it is initialized.";
	m_timerCounterPinOffset = offset;
}

int LabJackDevice::getTimerCounterPinOffset() const
{
	return m_timerCounterPinOffset;
}

void LabJackDevice::enableTimer(int index, LabJack::TimerMode mode)
{
	SURGSIM_ASSERT(!isInitialized()) << "Timers cannot be enabled for a LabJackDevice after it is initialized.";
	LabJack::TimerSettings timerModeAndOptionalInitialValue = {mode, DataStructures::OptionalValue<int>()};
	m_timers[index] = std::move(timerModeAndOptionalInitialValue);
}

void LabJackDevice::enableTimer(int index, LabJack::TimerMode mode, int initialValue)
{
	SURGSIM_ASSERT(!isInitialized()) << "Timers cannot be enabled for a LabJackDevice after it is initialized.";
	LabJack::TimerSettings timerModeAndOptionalInitialValue = {mode, DataStructures::OptionalValue<int>(initialValue)};
	m_timers[index] = std::move(timerModeAndOptionalInitialValue);
}

void LabJackDevice::setTimers(const std::unordered_map<int, LabJack::TimerSettings>& timers)
{
	SURGSIM_ASSERT(!isInitialized()) << "Timers cannot be enabled for a LabJackDevice after it is initialized.";
	m_timers = timers;
}

const std::unordered_map<int, LabJack::TimerSettings>& LabJackDevice::getTimers() const
{
	return m_timers;
}

void LabJackDevice::setMaximumUpdateRate(double rate)
{
	SURGSIM_ASSERT(!isInitialized()) <<
		"LabJackDevice's maximum update rate cannot be set after it is initialized.";
	m_threadRate = rate;
}

double LabJackDevice::getMaximumUpdateRate() const
{
	return m_threadRate;
}

void LabJackDevice::enableAnalogInput(int positiveChannel, LabJack::Range range, int negativeChannel)
{
	SURGSIM_ASSERT(!isInitialized()) <<
		"Analog inputs cannot be enabled for a LabJackDevice after it is initialized.";
	LabJack::AnalogInputSettings rangeAndOptionalNegativeChannel = {range,
		DataStructures::OptionalValue<int>(negativeChannel)};
	m_analogInputs[positiveChannel] = std::move(rangeAndOptionalNegativeChannel);
}

void LabJackDevice::enableAnalogInput(int channel, LabJack::Range range)
{
	SURGSIM_ASSERT(!isInitialized()) <<
		"Analog inputs cannot be enabled for a LabJackDevice after it is initialized.";
	LabJack::AnalogInputSettings rangeAndOptionalNegativeChannel = {range, DataStructures::OptionalValue<int>()};
	m_analogInputs[channel] = std::move(rangeAndOptionalNegativeChannel);
}

void LabJackDevice::setAnalogInputs(const std::unordered_map<int,
	LabJack::AnalogInputSettings>& analogInputs)
{
	SURGSIM_ASSERT(!isInitialized()) <<
		"Analog inputs cannot be enabled for a LabJackDevice after it is initialized.";
	m_analogInputs = analogInputs;
}

const std::unordered_map<int, LabJack::AnalogInputSettings>& LabJackDevice::getAnalogInputs() const
{
	return m_analogInputs;
}

void LabJackDevice::enableAnalogOutput(int channel)
{
	SURGSIM_ASSERT(!isInitialized()) << "Analog outputs cannot be enabled for a LabJackDevice after it is initialized.";
	m_analogOutputChannels.insert(channel);
}

void LabJackDevice::setAnalogOutputs(const std::unordered_set<int>& analogOutputChannels)
{
	SURGSIM_ASSERT(!isInitialized()) << "Analog outputs cannot be enabled for a LabJackDevice after it is initialized.";
	m_analogOutputChannels = analogOutputChannels;
}

const std::unordered_set<int>& LabJackDevice::getAnalogOutputs() const
{
	return m_analogOutputChannels;
}

void LabJackDevice::setAnalogInputResolution(int resolution)
{
	SURGSIM_ASSERT(!isInitialized()) <<
		"Analog input resolution cannot be set for a LabJackDevice after it is initialized.";
	m_analogInputResolution = resolution;
}

int LabJackDevice::getAnalogInputResolution() const
{
	return m_analogInputResolution;
}

void LabJackDevice::setAnalogInputSettling(int settling)
{
	SURGSIM_ASSERT(!isInitialized()) <<
		"Analog input settling time cannot be set for a LabJackDevice after it is initialized.";
	m_analogInputSettling = settling;
}

int LabJackDevice::getAnalogInputSettling() const
{
	return m_analogInputSettling;
}

};  // namespace Devices
};  // namespace SurgSim

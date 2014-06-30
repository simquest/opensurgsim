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
namespace Device
{

LabJackDevice::LabJackDevice(const std::string& uniqueName) :
	SurgSim::Input::CommonDevice(uniqueName, LabJackScaffold::buildDeviceInputData()),
	m_model(LabJack::MODEL_SEARCH),
	m_connection(LabJack::CONNECTION_SEARCH),
	m_address(""),
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

	if (getDigitalOutputChannels().size() > 0)
	{
		SURGSIM_ASSERT(hasOutputProducer()) << "LabJackDevice named " << getName() <<
			" has digital output channels. An output producer is required, call setOutputProducer.";
	}

	if (getAnalogOutputChannels().size() > 0)
	{
		SURGSIM_ASSERT(hasOutputProducer()) << "LabJackDevice named " << getName() <<
			" has analog output channels. An output producer is required, call setOutputProducer.";
	}

	if (getTimers().size() > 0)
	{
		SURGSIM_ASSERT(hasOutputProducer()) << "LabJackDevice named " << getName() <<
			" has timers. An output producer is required, call setOutputProducer.";
	}

	std::shared_ptr<LabJackScaffold> scaffold = LabJackScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold) << "LabJackDevice failed to get a LabJackScaffold.";

	bool found = false;
	// registerDevice will set this object's type and/or connection, if they are currently set to SEARCH.
	if (scaffold->registerDevice(this))
	{
		m_scaffold = std::move(scaffold);
		SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": Initialized.";
		found = true;
	}
	return found;
}

bool LabJackDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << "LabJackDevice has not been initialized before finalize.";
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Finalizing.";
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

void LabJackDevice::setDigitalInputChannels(const std::unordered_set<int>& digitalInputChannels)
{
	SURGSIM_ASSERT(!isInitialized()) << "Digital inputs cannot be set for a LabJackDevice after it is initialized.";
	m_digitalInputChannels = digitalInputChannels;
}

const std::unordered_set<int>& LabJackDevice::getDigitalInputChannels() const
{
	return m_digitalInputChannels;
}

void LabJackDevice::setDigitalOutputChannels(const std::unordered_set<int>& digitalOutputChannels)
{
	SURGSIM_ASSERT(!isInitialized()) << "Digital outputs cannot be set for a LabJackDevice after it is initialized.";
	m_digitalOutputChannels = digitalOutputChannels;
}

const std::unordered_set<int>& LabJackDevice::getDigitalOutputChannels() const
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

void LabJackDevice::setTimers(const std::unordered_map<int, LabJack::TimerMode>& timers)
{
	SURGSIM_ASSERT(!isInitialized()) << "Timers cannot be added to a LabJackDevice after it is initialized.";
	m_timers = timers;
}

const std::unordered_map<int, LabJack::TimerMode>& LabJackDevice::getTimers() const
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

void LabJackDevice::setAnalogInputsDifferential(std::unordered_map<int,
	LabJack::RangeAndOptionalNegativeChannel> analogInputs)
{
	SURGSIM_ASSERT(!isInitialized()) <<
		"Differential analog inputs cannot be set for a LabJackDevice after it is initialized.";
	m_analogInputsDifferential = analogInputs;
}

const std::unordered_map<int, LabJack::RangeAndOptionalNegativeChannel>& LabJackDevice::getAnalogInputsDifferential() const
{
	return m_analogInputsDifferential;
}

void LabJackDevice::setAnalogInputsSingleEnded(std::unordered_map<int, LabJack::Range> analogInputs)
{
	SURGSIM_ASSERT(!isInitialized()) <<
		"Single-ended analog inputs cannot be set for a LabJackDevice after it is initialized.";
	m_analogInputsSingleEnded = analogInputs;
}

const std::unordered_map<int, LabJack::Range>& LabJackDevice::getAnalogInputsSingleEnded() const
{
	return m_analogInputsSingleEnded;
}

void LabJackDevice::setAnalogOutputChannels(const std::unordered_set<int>& analogOutputChannels)
{
	SURGSIM_ASSERT(!isInitialized()) << "Analog outputs cannot be set for a LabJackDevice after it is initialized.";
	m_analogOutputChannels = analogOutputChannels;
}

const std::unordered_set<int>& LabJackDevice::getAnalogOutputChannels() const
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

};  // namespace Device
};  // namespace SurgSim

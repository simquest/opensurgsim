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

#include "SurgSim/Devices/Trakstar/TrakstarDevice.h"

#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Devices/Trakstar/TrakstarScaffold.h"

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::TrakstarDevice, TrakstarDevice);

TrakstarDevice::TrakstarDevice(const std::string& uniqueName) :
	Input::CommonDevice(uniqueName, TrakstarScaffold::buildDeviceInputData()),
	m_sensorId(0)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TrakstarDevice, unsigned short, SensorID, getSensorId, setSensorId);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TrakstarDevice, SurgSim::DataStructures::OptionalValue<double>, MeasurementRate,
		getOptionalMeasurementRate, setOptionalMeasurementRate);
}

TrakstarDevice::~TrakstarDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}

bool TrakstarDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized()) << "Trakstar device already initialized.";
	auto scaffold = TrakstarScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold != nullptr) << "TrakstarDevice::initialize(): Failed to obtain a Trakstar scaffold.";

	bool initialize = false;
	if (scaffold->registerDevice(this))
	{
		m_scaffold = std::move(scaffold);
		initialize = true;
	}
	return initialize;
}

bool TrakstarDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << "Trakstar device already finalized.";
	bool ok = m_scaffold->unregisterDevice(this);
	m_scaffold = nullptr;
	return ok;
}

bool TrakstarDevice::isInitialized() const
{
	return (nullptr != m_scaffold);
}


unsigned short TrakstarDevice::getSensorId() const
{
	return m_sensorId;
}

void TrakstarDevice::setSensorId(unsigned short id)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot set TrakstarDevice sensor ID after initialization!";
	m_sensorId = id;
}

DataStructures::OptionalValue<double> TrakstarDevice::getOptionalMeasurementRate() const
{
	return m_measurementRate;
}

void TrakstarDevice::setOptionalMeasurementRate(DataStructures::OptionalValue<double> rate)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot set TrakstarDevice measurement rate after initialization!";
	if (rate.hasValue())
	{
		auto val = rate.getValue();
		if (val > 765.0)
		{
			SURGSIM_LOG_WARNING(Framework::Logger::getLogger("Devices/Trakstar")) <<
				"Attempting to set TrakstarDevice measurement rate to " << val << ". The maximum rate is 765 Hz.";
			rate.setValue(765.0);
		}
		if (val < 60)
		{
			SURGSIM_LOG_WARNING(Framework::Logger::getLogger("Devices/Trakstar")) <<
				"Attempting to set TrakstarDevice measurement rate to " << val << ". The minimum rate is 60 Hz.";
			rate.setValue(60.0);
		}
	}
	m_measurementRate = rate;
}

void TrakstarDevice::setMeasurementRate(double rate)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot set TrakstarDevice measurement rate after initialization!";
	if (rate > 765.0)
	{
		SURGSIM_LOG_WARNING(Framework::Logger::getLogger("Devices/Trakstar")) <<
			"Attempting to set TrakstarDevice measurement rate to " << rate << ". The maximum rate is 765 Hz.";
		rate = 765.0;
	}
	if (rate < 60.0)
	{
		SURGSIM_LOG_WARNING(Framework::Logger::getLogger("Devices/Trakstar")) <<
			"Attempting to set TrakstarDevice measurement rate to " << rate << ". The minimum rate is 60 Hz.";
		rate = 60.0;
	}
	m_measurementRate.setValue(rate);
}

};  // namespace Devices
};  // namespace SurgSim

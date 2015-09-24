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

#include "SurgSim/Devices/Novint/NovintDevice.h"

#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/Devices/Novint/NovintScaffold.h"

using SurgSim::DataStructures::OptionalValue;

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::NovintDevice, NovintDevice);

NovintDevice::NovintDevice(const std::string& uniqueName) :
	Input::CommonDevice(uniqueName, NovintScaffold::buildDeviceInputData()),
	m_positionScale(1.0), m_orientationScale(1.0), m_7DofDevice(false)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, OptionalValue<std::string>, InitializationName,
		getOptionalInitializationName, setOptionalInitializationName);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, OptionalValue<std::string>, SerialNumber,
		getOptionalSerialNumber, setOptionalSerialNumber);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, bool, 7DofDevice, is7DofDevice, set7DofDevice);
}

NovintDevice::~NovintDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}

void NovintDevice::setSerialNumber(const std::string& serialNumber)
{
	SURGSIM_ASSERT(!m_initializationName.hasValue()) << "Cannot set serialNumber for a NovintDevice named " <<
		getName() << ", which already has an initializationName.";
	SURGSIM_ASSERT(!isInitialized()) <<
		"Cannot setSerialNumber after the device named " << getName() << " has been initialized.";
	m_serialNumber.setValue(serialNumber);
}

bool NovintDevice::getSerialNumber(std::string* serialNumber) const
{
	const bool hasValue = m_serialNumber.hasValue();
	if (hasValue)
	{
		*serialNumber = m_serialNumber.getValue();
	}
	return hasValue;
}

void NovintDevice::setInitializationName(const std::string& initializationName)
{
	SURGSIM_ASSERT(!m_serialNumber.hasValue()) << "Cannot set initializationName for a NovintDevice named " <<
		getName() << ", which already has a serialNumber.";
	SURGSIM_ASSERT(!isInitialized()) <<
		"Cannot setInitializationName after the device named " << getName() << " has been initialized.";
	m_initializationName.setValue(initializationName);
}

bool NovintDevice::getInitializationName(std::string* initializationName) const
{
	const bool hasValue = m_initializationName.hasValue();
	if (hasValue)
	{
		*initializationName = m_initializationName.getValue();
	}
	return hasValue;
}

bool NovintDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized());
	std::shared_ptr<NovintScaffold> scaffold = NovintScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold);

	if (!scaffold->registerDevice(this))
	{
		return false;
	}

	m_scaffold = std::move(scaffold);
	return true;
}

bool NovintDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized());
	bool result = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();
	return result;
}

bool NovintDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}

void NovintDevice::setPositionScale(double scale)
{
	m_positionScale = scale;
	if (m_scaffold)
	{
		m_scaffold->setPositionScale(this, m_positionScale);
	}
}

double NovintDevice::getPositionScale() const
{
	return m_positionScale;
}

void NovintDevice::setOrientationScale(double scale)
{
	m_orientationScale = scale;
	if (m_scaffold)
	{
		m_scaffold->setOrientationScale(this, m_orientationScale);
	}
}

double NovintDevice::getOrientationScale() const
{
	return m_orientationScale;
}

void NovintDevice::set7DofDevice(bool val)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot set 7Dof status after initialization.";
	m_7DofDevice = val;
}

bool NovintDevice::is7DofDevice() const
{
	return m_7DofDevice;
}

const OptionalValue<std::string>& NovintDevice::getOptionalInitializationName() const
{
	return m_initializationName;
}

void NovintDevice::setOptionalInitializationName(const OptionalValue<std::string>& name)
{
	m_initializationName = name;
}

const OptionalValue<std::string>& NovintDevice::getOptionalSerialNumber() const
{
	return m_serialNumber;
}

void NovintDevice::setOptionalSerialNumber(const OptionalValue<std::string>& serial)
{
	m_serialNumber = serial;
}

};  // namespace Devices
};  // namespace SurgSim

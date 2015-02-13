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

#include "SurgSim/Devices/Novint/NovintCommonDevice.h"

#include "SurgSim/Devices/Novint/NovintScaffold.h"

namespace SurgSim
{
namespace Device
{

NovintCommonDevice::NovintCommonDevice(const std::string& uniqueName) :
	SurgSim::Input::CommonDevice(uniqueName, NovintScaffold::buildDeviceInputData()),
	m_positionScale(1.0), m_orientationScale(1.0)
{
}

NovintCommonDevice::~NovintCommonDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}

void NovintCommonDevice::setSerialNumber(const std::string& serialNumber)
{
	SURGSIM_ASSERT(!m_initializationName.hasValue()) << "Cannot set serialNumber for a NovintCommonDevice named " <<
		getName() << ", which already has an initializationName.";
	SURGSIM_ASSERT(!isInitialized()) <<
		"Cannot setSerialNumber after the device named " << getName() << " has been initialized.";
	m_serialNumber.setValue(serialNumber);
}

bool NovintCommonDevice::getSerialNumber(std::string* serialNumber) const
{
	const bool hasValue = m_serialNumber.hasValue();
	if (hasValue)
	{
		*serialNumber = m_serialNumber.getValue();
	}
	return hasValue;
}

void NovintCommonDevice::setInitializationName(const std::string& initializationName)
{
	SURGSIM_ASSERT(!m_serialNumber.hasValue()) << "Cannot set initializationName for a NovintCommonDevice named " <<
		getName() << ", which already has a serialNumber.";
	SURGSIM_ASSERT(!isInitialized()) <<
		"Cannot setInitializationName after the device named " << getName() << " has been initialized.";
	m_initializationName.setValue(initializationName);
}

bool NovintCommonDevice::getInitializationName(std::string* initializationName) const
{
	const bool hasValue = m_initializationName.hasValue();
	if (hasValue)
	{
		*initializationName = m_initializationName.getValue();
	}
	return hasValue;
}

bool NovintCommonDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized());
	std::shared_ptr<NovintScaffold> scaffold = NovintScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold);

	if (! scaffold->registerDevice(this))
	{
		return false;
	}

	m_scaffold = std::move(scaffold);
	return true;
}

bool NovintCommonDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized());
	bool result = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();
	return result;
}

bool NovintCommonDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}

void NovintCommonDevice::setPositionScale(double scale)
{
	m_positionScale = scale;
	if (m_scaffold)
	{
		m_scaffold->setPositionScale(this, m_positionScale);
	}
}

double NovintCommonDevice::getPositionScale() const
{
	return m_positionScale;
}

void NovintCommonDevice::setOrientationScale(double scale)
{
	m_orientationScale = scale;
	if (m_scaffold)
	{
		m_scaffold->setOrientationScale(this, m_orientationScale);
	}
}

double NovintCommonDevice::getOrientationScale() const
{
	return m_orientationScale;
}

bool NovintCommonDevice::is7DofDevice() const
{
	return false;
}

};  // namespace Device
};  // namespace SurgSim

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
#include "SurgSim/Math/MathConvert.h"

using SurgSim::DataStructures::OptionalValue;

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::NovintDevice, NovintDevice);

NovintDevice::NovintDevice(const std::string& uniqueName) :
	Input::CommonDevice(uniqueName, NovintScaffold::buildDeviceInputData()),
	m_positionScale(1.0),
	m_orientationScale(1.0),
	m_7DofDevice(false),
	m_maxForce(8.9),
	m_antigrav(Math::Vector3d::Zero()),
	m_yawOffset(0.0),
	m_pitchOffset(0.0),
	m_rollOffset(0.0),
	m_toolDofOffset(0.0)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, OptionalValue<std::string>, InitializationName,
									  getOptionalInitializationName, setOptionalInitializationName);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, OptionalValue<std::string>, SerialNumber,
									  getOptionalSerialNumber, setOptionalSerialNumber);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, bool, 7DofDevice, is7DofDevice, set7DofDevice);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, double, PositionScale, getPositionScale, setPositionScale);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, double, OrientationScale, getOrientationScale, setOrientationScale);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, double, MaxForce, getMaxForce, setMaxForce);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, Math::Vector3d, Antigrav, getAntigrav, setAntigrav);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, double, YawOffset, getYawOffset, setYawOffset);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, double, PitchOffset, getPitchOffset, setPitchOffset);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, double, RollOffset, getRollOffset, setRollOffset);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(NovintDevice, double, ToolDofOffset, getToolDofOffset, setToolDofOffset);
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
									 "Cannot setSerialNumber after the device named " <<
									 getName() << " has been initialized.";
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
									 "Cannot setInitializationName after the device named " <<
									 getName() << " has been initialized.";
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
	SURGSIM_ASSERT(!isInitialized()) << getName() << " already initialized.";
	auto scaffold = NovintScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold != nullptr);

	bool initialize = false;
	if (scaffold->registerDevice(this))
	{
		m_scaffold = std::move(scaffold);
		initialize = true;
	}
	return initialize;
}

bool NovintDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << getName() << " is not initialized, cannot finalize.";
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

void NovintDevice::setMaxForce(double force)
{
	SURGSIM_ASSERT(!isInitialized()) <<
									 "Cannot setMaxForce after the device named " <<
									 getName() << " has been initialized.";
	SURGSIM_ASSERT(force >= 0.0) << "Cannot set a negative maximum force magnitude on device named " << getName();
	m_maxForce = force;
}

double NovintDevice::getMaxForce() const
{
	return m_maxForce;
}

void NovintDevice::setAntigrav(Math::Vector3d antigrav)
{
	SURGSIM_ASSERT(!isInitialized()) <<
									 "Cannot setAntigrav after the device named " <<
									 getName() << " has been initialized.";
	m_antigrav = antigrav;
}

Math::Vector3d NovintDevice::getAntigrav() const
{
	return m_antigrav;
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

double NovintDevice::getYawOffset() const
{
	return m_yawOffset;
}

void NovintDevice::setYawOffset(double yawOffset)
{
	m_yawOffset = yawOffset;
}

double NovintDevice::getPitchOffset() const
{
	return m_pitchOffset;
}

void NovintDevice::setPitchOffset(double pitchOffset)
{
	m_pitchOffset = pitchOffset;
}

double NovintDevice::getRollOffset() const
{
	return m_rollOffset;
}

void NovintDevice::setRollOffset(double rollOffset)
{
	m_rollOffset = rollOffset;
}

double NovintDevice::getToolDofOffset() const
{
	return m_toolDofOffset;
}

void NovintDevice::setToolDofOffset(double toolDofOffset)
{
	m_toolDofOffset = toolDofOffset;
}

};  // namespace Devices
};  // namespace SurgSim

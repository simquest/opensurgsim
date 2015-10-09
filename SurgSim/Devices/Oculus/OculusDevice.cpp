// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include "SurgSim/Devices/Oculus/OculusDevice.h"

#include "SurgSim/Devices/Oculus/OculusScaffold.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::OculusDevice, OculusDevice);

OculusDevice::OculusDevice(const std::string& name) :
	SurgSim::Input::CommonDevice(name, OculusScaffold::buildDeviceInputData()),
	m_nearPlane(0.1f),
	m_farPlane(10.0f)
{
}

OculusDevice::~OculusDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}


bool OculusDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized()) << getName() << " already initialized.";
	auto scaffold = OculusScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold != nullptr);

	bool initialize = false;
	if (scaffold->registerDevice(this))
	{
		m_scaffold = std::move(scaffold);
		initialize = true;
	}
	return initialize;
}

bool OculusDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << getName() << " is not initialized, cannot finalize.";
	bool result = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();
	return result;
}

bool OculusDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}

void OculusDevice::setNearPlane(float nearPlane)
{
	SURGSIM_ASSERT(nearPlane > 0.0f) << "Can not use a negative near plane.";
	SURGSIM_LOG_IF(nearPlane > m_farPlane, SurgSim::Framework::Logger::getLogger("Device/OculusDevice"), WARNING) <<
		__FUNCTION__ << "Trying to set a near plane that is greater than the far plane.";
	m_nearPlane = nearPlane;
}

float OculusDevice::getNearPlane() const
{
	return m_nearPlane;
}

void OculusDevice::setFarPlane(float farPlane)
{
	SURGSIM_ASSERT(farPlane > 0.0f) << "Can not use a negative far plane.";
	SURGSIM_LOG_IF(farPlane < m_nearPlane, SurgSim::Framework::Logger::getLogger("Device/OculusDevice"), WARNING) <<
		__FUNCTION__ << "Trying to set a far plane that is smaller than the near plane.";
	m_farPlane = farPlane;
}

float OculusDevice::getFarPlane() const
{
	return m_farPlane;
}

};  // namespace Devices
};  // namespace SurgSim

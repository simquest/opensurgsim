// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Input/InputComponent.h"

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Math/MathConvert.h"


namespace SurgSim
{
namespace Input
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Input::InputComponent, InputComponent);

InputComponent::InputComponent(const std::string& name) :
	Representation(name),
	m_toElementTransform(SurgSim::Math::RigidTransform3d::Identity()),
	m_hasInput(false)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(InputComponent, std::string, DeviceName,
		getDeviceName, setDeviceName);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(InputComponent, SurgSim::Math::RigidTransform3d, ToElementTransform,
		getToElementTransform, setToElementTransform);

	// We are using the localPose for this, so this property should not be explicitly serialized
	SURGSIM_ADD_RW_PROPERTY(InputComponent, SurgSim::Math::RigidTransform3d, ToDeviceTransform,
		getToElementTransform, setToDeviceTransform);
}

InputComponent::~InputComponent()
{
}

void InputComponent::setDeviceName(const std::string& deviceName)
{
	m_deviceName = deviceName;
}

std::string InputComponent::getDeviceName() const
{
	return m_deviceName;
}

void InputComponent::getData(SurgSim::DataStructures::DataGroup* dataGroup)
{
	if (m_hasInput)
	{
		m_lastInput.get(dataGroup);
	}
}

bool InputComponent::doInitialize()
{
	return true;
}

bool InputComponent::doWakeUp()
{
	return true;
}

void InputComponent::initializeInput(const std::string& device,
		const SurgSim::DataStructures::DataGroup& initialData)
{
	m_hasInput = true;
	m_lastInput.set(initialData);
}

void InputComponent::handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData)
{
	m_hasInput = true;
	m_lastInput.set(inputData);
}

SurgSim::Math::RigidTransform3d InputComponent::getToDeviceTransform() const
{
	return getLocalPose();
}

void InputComponent::setToDeviceTransform(const SurgSim::Math::RigidTransform3d& val)
{
	setLocalPose(val);
}

SurgSim::Math::RigidTransform3d InputComponent::getToElementTransform() const
{
	return m_toElementTransform;
}

void InputComponent::setToElementTransform(const SurgSim::Math::RigidTransform3d& val)
{
	m_toElementTransform = val;
}

}; // namespace Input
}; // namespace SurgSim


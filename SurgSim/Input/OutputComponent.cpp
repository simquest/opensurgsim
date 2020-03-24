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

#include "SurgSim/Input/OutputComponent.h"

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Math/MathConvert.h"


namespace SurgSim
{
namespace Input
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Input::OutputComponent, OutputComponent);

OutputComponent::OutputComponent(const std::string& name) :
	Representation(name),
	m_deviceName(),
	m_haveData(false),
	m_toElementTransform(SurgSim::Math::RigidTransform3d::Identity())
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(OutputComponent, std::string, DeviceName, getDeviceName, setDeviceName);


	SURGSIM_ADD_SERIALIZABLE_PROPERTY(OutputComponent, SurgSim::Math::RigidTransform3d, ToElementTransform,
		getToElementTransform, setToElementTransform);

	// We are using the localPose for this, so this property should not be explicitly serialized
	SURGSIM_ADD_RW_PROPERTY(OutputComponent, SurgSim::Math::RigidTransform3d, ToDeviceTransform,
		getToElementTransform, setToDeviceTransform);
}

OutputComponent::~OutputComponent()
{
}

void OutputComponent::setDeviceName(const std::string& deviceName)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot call OutputComponent::setDeviceName after initialization of "
		<< getFullName();
	m_deviceName = deviceName;
}

void OutputComponent::setData(const SurgSim::DataStructures::DataGroup& dataGroup)
{
	m_lastOutput.set(dataGroup);
	m_haveData = true;
}

bool OutputComponent::doInitialize()
{
	return true;
}

bool OutputComponent::doWakeUp()
{
	return true;
}

std::string OutputComponent::getDeviceName() const
{
	return m_deviceName;
}

bool OutputComponent::requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData)
{
	bool result = false;
	if (m_haveData && (outputData != nullptr))
	{
		m_lastOutput.get(outputData); // cannot get() until after the first call to setData
		result = true;
	}
	return result;
}

SurgSim::Math::RigidTransform3d OutputComponent::getToDeviceTransform() const
{
	return getLocalPose();
}

void OutputComponent::setToDeviceTransform(const SurgSim::Math::RigidTransform3d& val)
{
	setLocalPose(val);
}

SurgSim::Math::RigidTransform3d OutputComponent::getToElementTransform() const
{
	return m_toElementTransform;
}

void OutputComponent::setToElementTransform(const SurgSim::Math::RigidTransform3d& val)
{
	m_toElementTransform = val;
}

}; // namespace Input
}; // namespace SurgSim

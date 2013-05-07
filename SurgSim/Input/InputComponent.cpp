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

#include "InputComponent.h"

namespace SurgSim
{
namespace Input
{

InputComponent::InputComponent(std::string name, std::string deviceName) :
	Component(name),
	m_deviceName(deviceName),
	m_deviceConnected(false)
{

}

InputComponent::~InputComponent()
{

}

void InputComponent::deviceConnected(const std::string& device, const SurgSim::DataStructures::DataGroup& initialData)
{
	SURGSIM_ASSERT(initialData.isValid()) 
		<< "Cannot initialize input component (" << getName() << ") with invalid data from device (" << device << ")";
	m_lastInput.set(initialData);
	m_deviceConnected = true;
}

void InputComponent::deviceDisconnected(const std::string& device)
{
	m_deviceConnected = false;
}

void InputComponent::handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData)
{
	m_lastInput.set(inputData);
}

void InputComponent::getData(SurgSim::DataStructures::DataGroup* dataGroup)
{
	SURGSIM_ASSERT(m_deviceConnected) << "No device connected to " << getName() << ". Unable to getData.";
	m_lastInput.get(dataGroup);
}

bool InputComponent::doInitialize()
{
	return true;
}

bool InputComponent::doWakeUp()
{
	return true;
}

std::string InputComponent::getDeviceName() const
{
	return m_deviceName;
}


}; // namespace Input
}; // namespace SurgSim


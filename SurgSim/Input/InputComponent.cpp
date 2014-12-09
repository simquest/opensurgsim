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

#include "SurgSim/Input/InputComponent.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Input/InputConsumerInterface.h"

namespace SurgSim
{
namespace Input
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Input::InputComponent, InputComponent);

/// An input consumer monitors device and signal state update
class InputConsumer: public InputConsumerInterface
{
public:
	/// Constructor
	InputConsumer()
	{
	}
	/// Destructor
	virtual ~InputConsumer()
	{
	}

	/// Handle the input coming from device.
	/// \param device The name of the device that is producing the input.
	/// \param inputData The input data coming from the device.
	virtual void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override
	{
		m_lastInput.set(inputData);
	}

	/// Initialize the input data information stored in this input consumer.
	/// \param device The name of the device that is producing the input.
	/// \param initialData Initial input data of the device.
	virtual void initializeInput(const std::string& device,
		const SurgSim::DataStructures::DataGroup& initialData) override
	{
		m_lastInput.set(initialData);
	}

	/// Retrieve input data information stored in this input consumer
	/// \param [out] dataGroup Used to accept the retrieved input data information
	void getData(SurgSim::DataStructures::DataGroup* dataGroup)
	{
		m_lastInput.get(dataGroup);
	}

private:
	/// Used to store input data information passed in from device
	SurgSim::Framework::LockedContainer<SurgSim::DataStructures::DataGroup> m_lastInput;
};


InputComponent::InputComponent(const std::string& name) :
	Representation(name),
	m_deviceName(),
	m_deviceConnected(false),
	m_input(std::make_shared<InputConsumer>())
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(InputComponent, std::string, DeviceName,
		getDeviceName, setDeviceName);
}

InputComponent::~InputComponent()
{
}

void InputComponent::setDeviceName(const std::string& deviceName)
{
	m_deviceName = deviceName;
}

bool InputComponent::isDeviceConnected()
{
	return m_deviceConnected;
}

void InputComponent::getData(SurgSim::DataStructures::DataGroup* dataGroup)
{
	SURGSIM_ASSERT(m_deviceConnected) << "No device connected to InputComponent named '" << getName() <<
		"'. Unable to getData.";
	m_input->getData(dataGroup);
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

void InputComponent::connectDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device)
{
	device->addInputConsumer(m_input);
	m_deviceConnected = true;
}

void InputComponent::disconnectDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device)
{
	device->removeInputConsumer(m_input);
	m_deviceConnected = false;
}

}; // namespace Input
}; // namespace SurgSim


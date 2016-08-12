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

#include "SurgSim/Input/InputManager.h"

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/OutputComponent.h"

namespace SurgSim
{
namespace Input
{

InputManager::InputManager() :
	ComponentManager("Input Manager")
{
}

InputManager::~InputManager()
{
}

bool InputManager::doInitialize()
{
	return true;
}

bool InputManager::doStartUp()
{
	return true;
}

bool InputManager::doUpdate(double dt)
{
	processBehaviors(dt);
	processComponents();
	return true;
}

void InputManager::doBeforeStop()
{
	retireComponents(m_inputs);
	retireComponents(m_outputs);

	ComponentManager::doBeforeStop();
}

bool InputManager::executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	auto input = tryAddComponent(component, &m_inputs);
	if (input != nullptr)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		addInputComponent(input);
		return true;
	}

	auto output = tryAddComponent(component, &m_outputs);
	if (output != nullptr)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		addOutputComponent(output);
		return true;
	}

	// If we got he the component was neither an Input nor and OutputComponent, no add was performed
	// return false
	return false;
}

bool InputManager::executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	bool result = false;
	boost::lock_guard<boost::mutex> lock(m_mutex);
	if (tryRemoveComponent(component, &m_inputs))
	{
		auto input = std::static_pointer_cast<InputComponent>(component);
		DeviceInterface* device = nullptr;
		if (tryFindDevice(input->getDeviceName(), &device))
		{
			device->removeInputConsumer(input);
		}
		result = true;
	}
	else if (tryRemoveComponent(component, &m_outputs))
	{
		auto output = std::dynamic_pointer_cast<OutputComponent>(component);
		DeviceInterface* device = nullptr;
		if (tryFindDevice(output->getDeviceName(), &device))
		{
			device->removeOutputProducer(output);
		}
		result = true;
	}
	return result;
}

bool InputManager::addInputComponent(const std::shared_ptr<InputComponent>& input)
{
	bool result = false;
	DeviceInterface* device = nullptr;
	if (tryFindDevice(input->getDeviceName(), &device))
	{
		device->addInputConsumer(input);
		SURGSIM_LOG_INFO(m_logger) << "Added input component " << input->getFullName()
			<< " connected to device " << input->getDeviceName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_CRITICAL(m_logger) << " Could not find Device named '"
				<< input->getDeviceName() << "' when adding input component named '" << input->getFullName() << "'.";
	}
	return result;
}

bool InputManager::addOutputComponent(const std::shared_ptr<OutputComponent>& output)
{
	bool result = false;
	const auto outputName = output->getFullName();
	const auto deviceName = output->getDeviceName();
	DeviceInterface* device = nullptr;
	if (tryFindDevice(deviceName, &device))
	{
		if (device->hasOutputProducer())
		{
			SURGSIM_LOG_WARNING(m_logger) << "Trying to add OutputProducer " << outputName
				<< " to device " << deviceName
				<< ", but the device already has an OutputProducer assigned, this add will be ignored!";
		}
		else
		{
			device->setOutputProducer(output);
			SURGSIM_LOG_INFO(m_logger) << "Added output component " << outputName << " connected to device "
				<< deviceName;
			result = true;
		}
	}
	else
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Could not find Device with name " << deviceName
			<< " when adding output component " << outputName;
	}
	return result;
}

bool InputManager::addDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device)
{
	bool result = false;
	boost::lock_guard<boost::mutex> lock(m_mutex);
	DeviceInterface* foundDevice = nullptr;
	const auto& deviceName = device->getName();
	if (deviceName == "")
	{
		SURGSIM_LOG_WARNING(m_logger) << "Cannot add a device that has an empty name.";
	}
	else if (tryFindDevice(deviceName, &foundDevice))
	{
		SURGSIM_LOG_WARNING(m_logger) << "Device " << deviceName << " is already available in Input Manager";
	}
	else
	{
		m_devices[deviceName] = device;
		SURGSIM_LOG_INFO(m_logger) << "Added device " << deviceName;
		result = true;
	}
	return result;
}

bool InputManager::removeDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device)
{
	bool result = false;
	boost::lock_guard<boost::mutex> lock(m_mutex);
	auto it = m_devices.find(device->getName());
	if (it != m_devices.end())
	{
		m_devices.erase(it);
		SURGSIM_LOG_DEBUG(m_logger) << "Removed device " << device->getName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Failed to find device to remove, " << device->getName();
	}
	return result;
}

int InputManager::getType() const
{
	return SurgSim::Framework::MANAGER_TYPE_INPUT;
}

bool InputManager::tryFindDevice(const std::string& name, DeviceInterface** device)
{
	auto found = m_devices.find(name);
	if (found != m_devices.end())
	{
		*device = found->second.get();
		return true;
	}
	return false;
}

} // Input
} // SurgSim

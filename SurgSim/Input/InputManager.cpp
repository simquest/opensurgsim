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

#include "SurgSim/Input/InputManager.h"

#include <SurgSim/Framework/Component.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Input/InputComponent.h>

namespace SurgSim
{
namespace Input
{

InputManager::InputManager() :
	ComponentManager("Input Manager")
{
}

InputManager::~InputManager(void)
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
	// Add all components that came in before the last update
	processComponents();

	auto it = std::begin(m_behaviors);
	auto endIt = std::end(m_behaviors);
	for ( ;  it != endIt;  ++it)
	{
		(*it)->update(dt);
	}

	return true;
}

bool InputManager::executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component)
{

	auto input = tryAddComponent(component, &m_inputs);
	if (input != nullptr)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		// Early exit
		return addInputComponent(input);
	}

	auto output = tryAddComponent(component, &m_outputs);
	if (output != nullptr)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		// Early exit
		return addOutputComponent(output);
	}

	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		return tryAddComponent(component, &m_behaviors) != nullptr;
	}

	// If we got he the component was neither an Input nor and OutputComponent, no add was performed
	// return false
	return false;
}

bool InputManager::addInputComponent(const std::shared_ptr<InputComponent>& input)
{
	bool result = false;
	if (m_devices.find(input->getDeviceName()) != m_devices.end())
	{
		input->connectDevice(m_devices[input->getDeviceName()]);
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ <<
			                        " Added component " << input->getName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Could not find Device with name " <<
			                        input->getDeviceName() << " when adding component " << input->getName();
	}
	return result;
}

bool InputManager::addOutputComponent(const std::shared_ptr<OutputComponent>& output)
{
	bool result = false;
	if (m_devices.find(output->getDeviceName()) != m_devices.end())
	{
		if (! m_devices[output->getDeviceName()]->hasOutputProducer())
		{
			m_devices[output->getDeviceName()]->setOutputProducer(output);
			SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ <<
				" Added component " << output->getName();
			result = true;
		}
		else
		{
			SURGSIM_LOG_WARNING(m_logger) << __FUNCTION__ <<
				" Trying to add OutputProducer " << output->getName() << " to device " << output->getDeviceName() <<
				" but the device already has an OutputProducer assigned, this add will be ignored!";
		}
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Could not find Device with name " <<
			                        output->getDeviceName() << " when adding component " << output->getName();
	}
	return result;
}

bool InputManager::executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component)
{

	bool result = false;
	boost::lock_guard<boost::mutex> lock(m_mutex);
	if (tryRemoveComponent(component, &m_inputs))
	{
		auto input = std::static_pointer_cast<InputComponent>(component);
		input->disconnectDevice(m_devices[input->getDeviceName()]);
		result = true;
	}
	else if(tryRemoveComponent(component, &m_outputs))
	{
		auto output = std::dynamic_pointer_cast<OutputComponent>(component);
		m_devices[output->getDeviceName()]->setOutputProducer(nullptr);
		result = true;
	}
	return result;
}

bool InputManager::addDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device)
{
	bool result = false;
	boost::lock_guard<boost::mutex> lock(m_mutex);
	if (m_devices.find(device->getName()) == m_devices.cend())
	{
		m_devices[device->getName()] = device;
		result = true;
	}
	else
	{
		SURGSIM_LOG_WARNING(m_logger) << __FUNCTION__ << "Device already available in Input Manager";
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
		result = true;
	}
	return result;
}

} // Input
} // SurgSim

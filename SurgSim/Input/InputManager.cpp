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

#include "InputManager.h"

#include <SurgSim/Framework/Component.h>
#include <SurgSim/Framework/Runtime.h>

#include "InputComponent.h"

namespace SurgSim
{
namespace Input
{

InputManager::InputManager() :
	BasicThread("Input Manager")
{
	// Use a default logger, if there are errors before runtime is set here
	m_logger = SurgSim::Framework::Logger::createConsoleLogger(getName());
}

InputManager::~InputManager(void)
{
}

bool InputManager::doInitialize()
{
	// When the runtime has been set pull the runtimes logger for this manager
	m_logger = getRuntime()->getLogger(getName());
	return true;
}

bool InputManager::doStartUp()
{
	return true;
}

bool InputManager::doUpdate(double dt)
{
	return true;
}

bool InputManager::addComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	std::shared_ptr<InputComponent> input = std::dynamic_pointer_cast<InputComponent>(component);
	if (input != nullptr)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		// Early exit
		return addInputComponent(input);
	}

	std::shared_ptr<OutputComponent> output = std::dynamic_pointer_cast<OutputComponent>(component);
	if (output != nullptr)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		// Early exit
		return addOutputComponent(output);
	}

	// If we got he the component was neither an Input nor and OutputComponent
	return false;
}

bool InputManager::addInputComponent(std::shared_ptr<InputComponent> input)
{
	bool result = false;
	if (find(m_inputs.begin(), m_inputs.end(), input) == m_inputs.end())
	{
		if (m_devices.find(input->getDeviceName()) != m_devices.end())
		{
			input->connectDevice(m_devices[input->getDeviceName()]);
			m_inputs.push_back(input);
			SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ <<
			                           " Added component " << input->getName();
			result = true;
		}
		else
		{
			SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Could not find Device with name " <<
			                           input->getDeviceName() << " when adding component " << input->getName();
		}
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ <<
		                           " Duplicate Component" << input->getName();
	}
	return result;
}

bool InputManager::addOutputComponent(std::shared_ptr<OutputComponent> output)
{
	bool result = false;
	if (find(m_outputs.begin(), m_outputs.end(), output) == m_outputs.end())
	{
		if (m_devices.find(output->getDeviceName()) != m_devices.end())
		{
			if (! m_devices[output->getDeviceName()]->hasOutputProducer())
			{
				m_devices[output->getDeviceName()]->setOutputProducer(output);
				m_outputs.push_back(output);
				SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ <<
					" Added component " << output->getName();
				result = true;
			}
			else
			{
				SURGSIM_LOG_WARNING(m_logger) << __FUNCTION__ <<
					" Trying to add OutputProducer " << output->getName() << " to device " << output->getDeviceName() <<
					" but the device already has and OutputProducer assigned, this add will be ignored!";
			}
		}
		else
		{
			SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Could not find Device with name " <<
			                           output->getDeviceName() << " when adding component " << output->getName();
		}
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ <<
		                           " Duplicate Component" << output->getName();
	}
	return result;
}

bool InputManager::removeComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	bool result = false;
	std::shared_ptr<InputComponent> input = std::dynamic_pointer_cast<InputComponent>(component);
	if (input != nullptr)
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		auto found = std::find(m_inputs.begin(), m_inputs.end(), input);
		if (found != m_inputs.end())
		{
			input->disconnectDevice(m_devices[input->getDeviceName()]);
			m_inputs.erase(found);
			result = true;
		}
	}
	else
	{
		std::shared_ptr<OutputComponent> output = std::dynamic_pointer_cast<OutputComponent>(component);
		if (output != nullptr)
		{
			boost::lock_guard<boost::mutex> lock(m_mutex);
			auto found = std::find(m_outputs.begin(), m_outputs.end(), output);
			if (found != m_outputs.end())
			{
				m_devices[output->getDeviceName()]->setOutputProducer(nullptr);
				m_outputs.erase(found);
				result = true;
			}
		}
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

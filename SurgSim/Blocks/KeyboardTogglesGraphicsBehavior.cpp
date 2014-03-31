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

#include "SurgSim/Blocks/KeyboardTogglesGraphicsBehavior.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Input/InputComponent.h"

namespace SurgSim
{
namespace Blocks
{

KeyboardTogglesGraphicsBehavior::KeyboardTogglesGraphicsBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_keyPressedLastUpdate(false)
{
}

void KeyboardTogglesGraphicsBehavior::setInputComponent(std::shared_ptr<SurgSim::Input::InputComponent> inputComponent)
{
	m_inputComponent = inputComponent;
}

void KeyboardTogglesGraphicsBehavior::registerKey(SurgSim::Device::KeyCode key,
												  std::shared_ptr<SurgSim::Framework::Component> component)
{
	auto graphicsRepresentation = std::dynamic_pointer_cast<SurgSim::Graphics::Representation>(component);
	if (nullptr != graphicsRepresentation)
	{
		m_register[static_cast<int>(key)].insert(graphicsRepresentation);
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"KeyboardTogglesGraphicsBehavior::registerKey(): Can not register component " << component->getName() << 
			". It's not a SurgSim::Graphics::Representation.\n";
	}
}

void KeyboardTogglesGraphicsBehavior::update(double dt)
{
	SurgSim::DataStructures::DataGroup dataGroup;
	m_inputComponent->getData(&dataGroup);

	int key;
	if (dataGroup.integers().get("key", &key))
	{
		auto match = m_register.find(key);
		if (match != m_register.end() && !m_keyPressedLastUpdate)
		{
			for (auto it = std::begin(match->second); it != std::end(match->second); ++it)
			{
				(*it)->setVisible(!(*it)->isVisible());
			};
		}
		m_keyPressedLastUpdate = (SurgSim::Device::KeyCode::NONE != key);
	}
}

bool KeyboardTogglesGraphicsBehavior::doInitialize()
{
	return true;
}

bool KeyboardTogglesGraphicsBehavior::doWakeUp()
{
	if (nullptr == m_inputComponent)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << "KeyboardTogglesGraphicsBehavior " <<
			getName() << " does not have an Input Component.";
		return false;
	}
	return true;
}

}; // namespace Blocks

}; // namespace SurgSim

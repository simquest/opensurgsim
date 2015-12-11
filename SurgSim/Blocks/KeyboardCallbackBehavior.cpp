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

#include "SurgSim/Blocks/KeyboardCallbackBehavior.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Input/InputComponent.h"

namespace SurgSim
{
namespace Blocks
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::KeyboardCallbackBehavior, KeyboardCallbackBehavior);

KeyboardCallbackBehavior::KeyboardCallbackBehavior(const std::string& name) :
	Framework::Behavior(name),
	m_keyPressedLastUpdate(false),
	m_actionKey(Devices::NONE)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(KeyboardCallbackBehavior, std::shared_ptr<SurgSim::Framework::Component>,
										InputComponent, getInputComponent, setInputComponent);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(KeyboardCallbackBehavior, int, ActionKey, getKey, setKey);
}

void KeyboardCallbackBehavior::setInputComponent(std::shared_ptr<Framework::Component> inputComponent)
{
	m_inputComponent = Framework::checkAndConvert<Input::InputComponent>(inputComponent,
													"SurgSim::Input::InputComponent");
}

std::shared_ptr<Input::InputComponent> KeyboardCallbackBehavior::getInputComponent() const
{
	return m_inputComponent;
}

void KeyboardCallbackBehavior::registerKey(Devices::KeyCode key)
{
	m_actionKey = key;
}

void KeyboardCallbackBehavior::update(double dt)
{
	DataStructures::DataGroup dataGroup;
	m_inputComponent->getData(&dataGroup);

	int key;
	if (dataGroup.integers().get("key", &key))
	{
		if (m_actionKey == key && !m_keyPressedLastUpdate)
		{
			m_callback();
		}

		m_keyPressedLastUpdate = (Devices::KeyCode::NONE != key);
	}
}

bool KeyboardCallbackBehavior::doInitialize()
{
	return true;
}

bool KeyboardCallbackBehavior::doWakeUp()
{
	bool result = true;
	if (nullptr == m_inputComponent)
	{
		SURGSIM_LOG_SEVERE(Framework::Logger::getDefaultLogger()) <<
			"KeyboardCallbackBehavior '" << getFullName() << "' does not have an Input Component.";
		result = false;
	}
	return result;
}

void KeyboardCallbackBehavior::setKey(int key)
{
	m_actionKey = key;
}

int KeyboardCallbackBehavior::getKey() const
{
	return m_actionKey;
}

void KeyboardCallbackBehavior::registerCallback(CallbackType func)
{
	m_callback = func;
}

}; // namespace Blocks
}; // namespace SurgSim
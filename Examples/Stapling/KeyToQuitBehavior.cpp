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

#include "Examples/Stapling/KeyToQuitBehavior.h"

#include <SurgSim/Input/InputComponent.h>
#include <SurgSim/DataStructures/DataGroup.h>


KeyToQuitBehavior::KeyToQuitBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_quitKey(SurgSim::Device::KEY_Q)
{

}

KeyToQuitBehavior::~KeyToQuitBehavior()
{

}

void KeyToQuitBehavior::update(double dt)
{
	SurgSim::DataStructures::DataGroup dataGroup;
	m_inputComponent->getData(&dataGroup);

	int key;
	if (dataGroup.integers().get("key", &key))
	{
		if ((m_quitKey == key || SurgSim::Device::KEY_ESCAPE == key)  && m_callback != nullptr)
		{
			m_callback(0);
		}
	}
}

bool KeyToQuitBehavior::doInitialize()
{
	SURGSIM_ASSERT(m_inputComponent != nullptr) << "'inputComponent' cannot be 'nullptr'";
	return true;
}

bool KeyToQuitBehavior::doWakeUp()
{
	return true;
}

void KeyToQuitBehavior::setInputComponent(std::shared_ptr<SurgSim::Framework::Component> inputComponent)
{
	SURGSIM_ASSERT(inputComponent != nullptr) << "'inputComponent' cannot be 'nullptr'";

	m_inputComponent = SurgSim::Framework::checkAndConvert<SurgSim::Input::InputComponent>(
						   inputComponent, "SurgSim::Input::InputComponent");
}

std::shared_ptr<SurgSim::Input::InputComponent> KeyToQuitBehavior::getInputComponent() const
{
	return m_inputComponent;
}

char KeyToQuitBehavior::getQuitKey() const
{
	return m_quitKey;
}

void KeyToQuitBehavior::setQuitKey(int val)
{
	m_quitKey = static_cast<SurgSim::Device::KeyCode>(val);
}

void KeyToQuitBehavior::setCallback(CallbackType callback)
{
	m_callback = callback;
}


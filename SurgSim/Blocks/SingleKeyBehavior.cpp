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

#include "SingleKeyBehavior.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Input/InputComponent.h"

namespace SurgSim
{
namespace Blocks
{

SingleKeyBehavior::SingleKeyBehavior(const std::string& name) :
	Framework::Behavior(name),
	m_actionKey(SurgSim::Devices::KeyCode::NONE),
	m_keyPressedLastUpdate(false)

{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SingleKeyBehavior, std::shared_ptr<SurgSim::Framework::Component>,
									  InputComponent, getInputComponent, setInputComponent);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SingleKeyBehavior, int, ActionKey, getKey, setKey);

}

SingleKeyBehavior::~SingleKeyBehavior()
{
}

void SingleKeyBehavior::setInputComponent(std::shared_ptr<SurgSim::Framework::Component> inputComponent)
{
	m_inputComponent = SurgSim::Framework::checkAndConvert<SurgSim::Input::InputComponent>(inputComponent,
					   "SurgSim::Input::InputComponent");
}

std::shared_ptr<SurgSim::Input::InputComponent> SingleKeyBehavior::getInputComponent() const
{
	return m_inputComponent;
}

bool SingleKeyBehavior::doInitialize()
{
	return true;
}

bool SingleKeyBehavior::doWakeUp()
{
	SURGSIM_LOG_IF(m_inputComponent == nullptr,
				   SurgSim::Framework::Logger::getDefaultLogger(),
				   WARNING) << "Input component not present in " << getFullName();
	SURGSIM_LOG_IF(m_actionKey != Devices::KeyCode::NONE,
				   SurgSim::Framework::Logger::getDefaultLogger(),
				   WARNING) << "No key set in " << getFullName();

	return m_inputComponent != nullptr && m_actionKey != Devices::KeyCode::NONE;
}

void SingleKeyBehavior::update(double dt)
{
	DataStructures::DataGroup dataGroup;
	m_inputComponent->getData(&dataGroup);

	int key;
	if (dataGroup.integers().get("key", &key))
	{
		if (getKey() == key && !m_keyPressedLastUpdate)
		{
			onKey(key);
		}
		m_keyPressedLastUpdate = (Devices::KeyCode::NONE != key);
	}
}

int SingleKeyBehavior::getKey() const
{
	return m_actionKey;
}

void SingleKeyBehavior::setKey(int val)
{
	m_actionKey = val;
}

}; // namespace Blocks
}; // namespace SurgSim
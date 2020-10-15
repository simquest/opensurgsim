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

#include "SurgSim/Blocks/KeyBehavior.h"

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

KeyBehavior::KeyBehavior(const std::string& name) :
	Framework::Behavior(name),
	m_lastKey(SurgSim::Devices::KeyCode::NONE)

{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(KeyBehavior, std::shared_ptr<SurgSim::Framework::Component>,
									  InputComponent, getInputComponent, setInputComponent);
}

KeyBehavior::~KeyBehavior()
{
}

void KeyBehavior::setInputComponent(std::shared_ptr<SurgSim::Framework::Component> inputComponent)
{
	m_inputComponent = SurgSim::Framework::checkAndConvert<SurgSim::Input::InputComponent>(inputComponent,
					   "SurgSim::Input::InputComponent");
}

std::shared_ptr<SurgSim::Input::InputComponent> KeyBehavior::getInputComponent() const
{
	return m_inputComponent;
}

bool KeyBehavior::doInitialize()
{
	return true;
}

bool KeyBehavior::doWakeUp()
{
	SURGSIM_LOG_IF(m_inputComponent == nullptr,
				   SurgSim::Framework::Logger::getDefaultLogger(),
				   WARNING) << "Input component not present in " << getFullName();

	return m_inputComponent != nullptr;
}

void KeyBehavior::update(double dt)
{
	DataStructures::DataGroup dataGroup;
	m_inputComponent->getData(&dataGroup);

	int key;
	if (dataGroup.integers().get("key", &key))
	{
		if (key != m_lastKey)
		{
			if (m_lastKey != Devices::KeyCode::NONE)
			{
				onKeyUp(m_lastKey);
			}

			if (key != Devices::KeyCode::NONE)
			{
				onKeyDown(key);
			}
		}
		m_lastKey = key;
	}
}

std::unordered_map<int, std::string> KeyBehavior::m_keyMap;
boost::mutex KeyBehavior::m_keyMapMutex;

bool KeyBehavior::registerKey(int keycode, const std::string& description)
{
	if (keycode == Devices::KeyCode::NONE)
	{
		return false;
	}
	bool result = false;
	boost::unique_lock<boost::mutex> lock(m_keyMapMutex);
	if (m_keyMap.find(keycode) == m_keyMap.end())
	{
		m_keyMap[keycode] = description;
		result = true;
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Blocks/KeyBehavior"))
				<< "Key '" << static_cast<char>(keycode) << "' [" << description << "] already registered for function "
				<< m_keyMap[keycode];
	}
	return result;
}

bool KeyBehavior::unregisterKey(int keycode)
{
	bool result = false;
	boost::unique_lock<boost::mutex> lock(m_keyMapMutex);
	if (m_keyMap.find(keycode) != m_keyMap.end())
	{
		m_keyMap.erase(keycode);
		result = true;
	}
	return result;
}

void KeyBehavior::logMap()
{
	std::string message;
	auto logger = SurgSim::Framework::Logger::getLogger("Blocks/KeyBehavior");
	boost::unique_lock<boost::mutex> lock(m_keyMapMutex);
	for (const auto& item : m_keyMap)
	{
		SURGSIM_LOG_INFO(logger) << "'" << static_cast<char>(item.first) << "' : " << item.second;
	}
}

void KeyBehavior::clearKeyMap()
{
	boost::unique_lock<boost::mutex> lock(m_keyMapMutex);
	m_keyMap.clear();
}

}; // namespace Blocks
}; // namespace SurgSim
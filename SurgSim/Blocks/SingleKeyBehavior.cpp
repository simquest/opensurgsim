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
	KeyBehavior(name),
	m_actionKey(SurgSim::Devices::KeyCode::NONE)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SingleKeyBehavior, int, ActionKey, getKey, setKey);

}

SingleKeyBehavior::~SingleKeyBehavior()
{
}

bool SingleKeyBehavior::doWakeUp()
{
	SURGSIM_LOG_IF(m_actionKey != Devices::KeyCode::NONE,
				   SurgSim::Framework::Logger::getDefaultLogger(),
				   WARNING) << "No key set in " << getFullName();

	return KeyBehavior::doWakeUp() && m_actionKey != Devices::KeyCode::NONE;
}


int SingleKeyBehavior::getKey() const
{
	return m_actionKey;
}

void SingleKeyBehavior::setDescription(const std::string& description)
{
	m_description = description;
}

void SingleKeyBehavior::onKeyDown(int actualKey)
{
	if (actualKey == m_actionKey)
	{
		onKey();
	}
}

void SingleKeyBehavior::onKeyUp(int)
{
	return;
}

void SingleKeyBehavior::setKey(int val)
{
	KeyBehavior::unregisterKey(m_actionKey);
	m_actionKey = val;
	std::string description = m_description;
	if (description == "")
	{
		description = getClassName();
	}
	KeyBehavior::registerKey(val, description);
}

}; // namespace Blocks
}; // namespace SurgSim
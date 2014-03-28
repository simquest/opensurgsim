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
#include "SurgSim/Input/InputComponent.h"

namespace SurgSim
{
namespace Blocks
{


KeyboardTogglesGraphicsBehavior::KeyboardTogglesGraphicsBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name)
{
}

void KeyboardTogglesGraphicsBehavior::setInputComponent(std::shared_ptr<SurgSim::Input::InputComponent> inputComponent)
{
	m_inputComponent = inputComponent;
}

// Note: This behavior is currently updated by BehaviorManager which runs at 30Hz.
// Since the speed (30Hz) is fast compared with the speed one key is pressed, the graphical representations will be
// set to visible/invisible even with one key press.
void KeyboardTogglesGraphicsBehavior::update(double dt)
{
	bool keyPressed = false;
	SurgSim::DataStructures::DataGroup dataGroup;
	m_inputComponent->getData(&dataGroup);

	int key;
	if (dataGroup.integers().get("key", &key))
	{
		auto match = m_keyRegister.find(key);
		if (match != m_keyRegister.end() && !keyPressed)
		{
			for (auto it = std::begin(match->second); it != std::end(match->second); ++it)
			{
				(*it)->setVisible(!(*it)->isVisible());
			};
		}
		keyPressed = (SurgSim::Device::KeyCode::NONE != key);
	}
}

bool KeyboardTogglesGraphicsBehavior::doInitialize()
{
	return true;
}

bool KeyboardTogglesGraphicsBehavior::doWakeUp()
{
	return true;
}

}; // namespace Blocks

}; // namespace SurgSim

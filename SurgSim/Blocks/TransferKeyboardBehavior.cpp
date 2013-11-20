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

#include <SurgSim/Blocks/TransferKeyboardBehavior.h>

#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/Devices/Keyboard/KeyboardCode.h>
#include <SurgSim/Framework/Representation.h>
#include <SurgSim/Input/InputComponent.h>

namespace SurgSim
{
namespace Blocks
{

TransferKeyboardBehavior::TransferKeyboardBehavior(const std::string& name,
													 std::shared_ptr<SurgSim::Input::InputComponent> from,
													 std::shared_ptr<SurgSim::Framework::Representation> to) :
	SurgSim::Framework::Behavior(name),	m_from(from), m_to(to)
{
}

void TransferKeyboardBehavior::update(double dt)
{
	SurgSim::DataStructures::DataGroup dataGroup;
	m_from->getData(&dataGroup);

	int key;
	dataGroup.integers().get("key", &key);

	auto pose = m_to->getPose();
	switch(key)
	{
	case SurgSim::Device::KeyCode::KEY_Left:
		std::cerr << "Left Arrow key pressed" << std::endl;
		break;
	case SurgSim::Device::KeyCode::KEY_Up:
		std::cerr << "Up Arrow key pressed" << std::endl;
		break;
	case SurgSim::Device::KeyCode::KEY_Right:
		std::cerr << "Right Arrow key pressed" << std::endl;
		break;
	case SurgSim::Device::KeyCode::KEY_Down:
		std::cerr << "Down Arrow key pressed" << std::endl;
		break;
	default:
		break;
	}
	//m_to->setPose(pose);
}

bool TransferKeyboardBehavior::doInitialize()
{
	return true;
}

bool TransferKeyboardBehavior::doWakeUp()
{
	return true;
}

}; //namespace Blocks
}; //namespace SurgSim

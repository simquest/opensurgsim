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

#include "Examples/StaplingDemo/AddStapleBehavior.h"

#include "SurgSim/Blocks/StapleElement.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/Scene.h"

#include <sstream>

using SurgSim::Math::RigidTransform3d;

AddStapleFromInputBehavior::AddStapleFromInputBehavior(
	const std::string& name):
SurgSim::Framework::Behavior(name), m_numElements(0), m_buttonPreviouslyPressed(false)
{
}

void AddStapleFromInputBehavior::setInputComponent(std::shared_ptr<SurgSim::Input::InputComponent> sender)
{
	m_from = sender;
}

void AddStapleFromInputBehavior::update(double dt)
{
	// Get the pose information from input device
	SurgSim::DataStructures::DataGroup dataGroup;
	m_from->getData(&dataGroup);

	RigidTransform3d pose;
	dataGroup.poses().get("pose", &pose);

	// Add staple to the scene from input
	bool button1 = false;
	dataGroup.booleans().get("button1", &button1);

	if (button1 && ! m_buttonPreviouslyPressed)
	{
		std::stringstream elementCount;
		elementCount << ++ m_numElements;

		std::string name = "stapleId_" + elementCount.str();

		// Create a staple element and add its into scene.
		std::shared_ptr<SurgSim::Framework::SceneElement> m_element =
			std::make_shared<SurgSim::Blocks::StapleElement>(name);

		std::static_pointer_cast<SurgSim::Blocks::StapleElement>(m_element)->setPose(pose);

		getScene()->addSceneElement(m_element);
	}
	m_buttonPreviouslyPressed = button1;
}

int AddStapleFromInputBehavior::getTargetManagerType() const
{
	return SurgSim::Framework::MANAGER_TYPE_INPUT;
}

bool AddStapleFromInputBehavior::doInitialize()
{
	return true;
}

bool AddStapleFromInputBehavior::doWakeUp()
{
	return true;
}

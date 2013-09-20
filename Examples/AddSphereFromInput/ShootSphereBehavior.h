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

#ifndef EXAMPLES_ADDSPHEREFROMINPUT_SHOOTSPHEREBEHAVIOR_H
#define EXAMPLES_ADDSPHEREFROMINPUT_SHOOTSPHEREBEHAVIOR_H

#include <SurgSim/Blocks/SphereElement.h>
#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/Framework/Behavior.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Input/InputComponent.h>
#include <SurgSim/Framework/Scene.h>

using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{

namespace Input
{

class ShootSphereFromInputBehavior: public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	/// \param	from	Input component to get the pose
	ShootSphereFromInputBehavior(const std::string& name, std::shared_ptr<SurgSim::Input::InputComponent> from):
		SurgSim::Framework::Behavior(name), m_from(from)
	{
	}

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt)
	{
		SurgSim::DataStructures::DataGroup dataGroup;
		m_from->getData(&dataGroup);
		RigidTransform3d pose;
		dataGroup.poses().get("pose", &pose);

		// Dynamically add sphere to the scene from input
		static int m_numElements = 0;
		bool button1;
		dataGroup.booleans().get("button1", &button1);
		if( button1 )
		{
			std::string name = "sphereId_" + m_numElements++;
			std::shared_ptr<SurgSim::Framework::SceneElement> m_element =
				std::make_shared<SurgSim::Blocks::SphereElement>(name, pose);

			getScene()->addSceneElement(m_element);
		}
	}

protected:
	/// Initialize the behavior
	virtual bool doInitialize()
	{
		return true;
	}
	/// Wakeup the behavior, which copies the initial pose
	virtual bool doWakeUp()
	{
		return true;
	}

private:
	/// Input component to get the pose
	std::shared_ptr<SurgSim::Input::InputComponent> m_from;
};

};  // namespace Blocks

};  // namespace SurgSim

#endif  // EXAMPLES_ADDSPHEREFROMINPUT_SHOOTSPHEREBEHAVIOR_H

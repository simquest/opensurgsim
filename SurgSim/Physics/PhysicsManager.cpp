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

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Physics/FreeMotionStep.h>
#include <SurgSim/Physics/Actors/RigidActorBase.h>
#include <SurgSim/Framework/Log.h>

namespace SurgSim
{
namespace Physics
{

PhysicsManager::PhysicsManager()
{

}

PhysicsManager::~PhysicsManager()
{

}

bool PhysicsManager::doInitialize()
{
	m_logger = getRuntime()->getLogger("PhysicsManager");
	m_rigidActors = std::make_shared<std::vector<std::shared_ptr<RigidActorBase>>>();
	m_freeMotionStep.reset(new FreeMotionStep(m_rigidActors));
	return m_logger != nullptr && m_rigidActors != nullptr && m_freeMotionStep != nullptr;
}

bool PhysicsManager::addComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	bool result = true;
	std::shared_ptr<RigidActorBase> actor = std::dynamic_pointer_cast<RigidActorBase>(component);
	if (actor != nullptr)
	{
		if (std::find(m_rigidActors->cbegin(), m_rigidActors->cend(), actor) != m_rigidActors->cend())
		{
			m_rigidActors->push_back(actor);
		}
		else
		{
			SURGSIM_LOG_DEBUG(m_logger) << __FUNCTION__ << " Object " << actor->getName() << " alread added to PhysicsManager";
			result = false;
		}
	}
	return result;
}

bool PhysicsManager::removeComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	bool result = false;

	return result;
}

bool PhysicsManager::doUpdate(double dt)
{
	m_freeMotionStep->step(dt);
	return true;
}


}; // Physics
}; // SurgSim

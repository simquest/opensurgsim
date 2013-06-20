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
#include <SurgSim/Framework/Component.h>
#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Physics/FreeMotion.h>
#include <SurgSim/Physics/Representation.h>
#include <SurgSim/Physics/DcdCollision.h>
#include <SurgSim/Framework/Log.h>


#include <list>

namespace SurgSim
{
namespace Physics
{

PhysicsManager::PhysicsManager() :
  ComponentManager("Physics Manager")
{

}

PhysicsManager::~PhysicsManager()
{

}

bool PhysicsManager::doInitialize()
{
	m_freeMotionStep.reset(new FreeMotion());
	m_dcdCollision.reset(new DcdCollision());
	return m_logger != nullptr;
}


bool PhysicsManager::doStartUp()
{
	return true;
}


bool PhysicsManager::executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	return tryAddComponent(component,&m_representations) != nullptr;
}

bool PhysicsManager::executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	return tryRemoveComponent(component, &m_representations);
}

bool PhysicsManager::doUpdate(double dt)
{
	// Add all components that came in before the last update
	processComponents();

	for (auto it = m_representations.begin(); it != m_representations.end(); ++it)
	{
		(*it)->beforeUpdate(dt);
	}

	std::list<std::shared_ptr<PhysicsManagerState>> stateList;
	std::shared_ptr<PhysicsManagerState> state = std::make_shared<PhysicsManagerState>();
	stateList.push_back(state);
	state->setRepresentations(m_representations);
	stateList.push_back(m_freeMotionStep->update(dt, stateList.back()));
	stateList.push_back(m_dcdCollision->update(dt, stateList.back()));

	for (auto it = m_representations.begin(); it != m_representations.end(); ++it)
	{
		(*it)->afterUpdate(dt);
	}

	return true;
}


}; // Physics
}; // SurgSim

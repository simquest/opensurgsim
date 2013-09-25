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

#include <SurgSim/Physics/PhysicsManager.h>

#include <SurgSim/Framework/Component.h>
#include <SurgSim/Physics/BuildMlcp.h>
#include <SurgSim/Physics/ContactConstraintGeneration.h>
#include <SurgSim/Physics/DcdCollision.h>
#include <SurgSim/Physics/FreeMotion.h>
#include <SurgSim/Physics/PhysicsManagerState.h>
#include <SurgSim/Physics/PostUpdate.h>
#include <SurgSim/Physics/PreUpdate.h>
#include <SurgSim/Physics/PushResults.h>
#include <SurgSim/Physics/Representation.h>
#include <SurgSim/Physics/SolveMlcp.h>

#include <list>

namespace SurgSim
{
namespace Physics
{

PhysicsManager::PhysicsManager() :
  ComponentManager("Physics Manager")
{
	setRate(1000.0);
}

PhysicsManager::~PhysicsManager()
{

}

int PhysicsManager::getType() const
{
	return SurgSim::Framework::MANAGER_TYPE_PHYSICS;
}

bool PhysicsManager::doInitialize()
{
	initializeComputations(false);
	return m_logger != nullptr;
}


bool PhysicsManager::doStartUp()
{
	return true;
}


bool PhysicsManager::executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	std::shared_ptr<Representation> representation = tryAddComponent(component, &m_representations);
	std::shared_ptr<CollisionRepresentation> collisionRep= tryAddComponent(component, &m_collisionRepresentations);
	return representation != nullptr || collisionRep != nullptr;
}

bool PhysicsManager::executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	bool removed1 = tryRemoveComponent(component, &m_representations);
	bool removed2 = tryRemoveComponent(component, &m_collisionRepresentations);
	return removed1 || removed2;
}

bool PhysicsManager::doUpdate(double dt)
{
	dt = 1e-3;

	// Add all components that came in before the last update
	processComponents();

	std::list<std::shared_ptr<PhysicsManagerState>> stateList;
	std::shared_ptr<PhysicsManagerState> state = std::make_shared<PhysicsManagerState>();
	stateList.push_back(state);
	state->setRepresentations(m_representations);
	state->setCollisionRepresentations(m_collisionRepresentations);

	stateList.push_back(m_preUpdateStep->update(dt, stateList.back()));
	stateList.push_back(m_freeMotionStep->update(dt, stateList.back()));
	stateList.push_back(m_dcdCollisionStep->update(dt, stateList.back()));
	stateList.push_back(m_constraintGenerationStep->update(dt, stateList.back()));
	stateList.push_back(m_buildMlcpStep->update(dt, stateList.back()));
	stateList.push_back(m_solveMlcpStep->update(dt, stateList.back()));
	stateList.push_back(m_pushResultsStep->update(dt, stateList.back()));
	stateList.push_back(m_postUpdateStep->update(dt, stateList.back()));

	return true;
}

void PhysicsManager::initializeComputations(bool copyState)
{
	m_preUpdateStep.reset(new PreUpdate(copyState));
	m_freeMotionStep.reset(new FreeMotion(copyState));
	m_dcdCollisionStep.reset(new DcdCollision(copyState));
	m_constraintGenerationStep.reset(new ContactConstraintGeneration(copyState));
	m_buildMlcpStep.reset(new BuildMlcp(copyState));
	m_solveMlcpStep.reset(new SolveMlcp(copyState));
	m_pushResultsStep.reset(new PushResults(copyState));
	m_postUpdateStep.reset(new PostUpdate(copyState));
}


}; // Physics
}; // SurgSim

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

#include "SurgSim/Physics/PhysicsManager.h"

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Physics/BuildMlcp.h"
#include "SurgSim/Physics/ConstraintComponent.h"
#include "SurgSim/Physics/ContactConstraintGeneration.h"
#include "SurgSim/Physics/DcdCollision.h"
#include "SurgSim/Physics/FreeMotion.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/PostUpdate.h"
#include "SurgSim/Physics/PreUpdate.h"
#include "SurgSim/Physics/PushResults.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/SolveMlcp.h"
#include "SurgSim/Physics/UpdateCollisionRepresentations.h"

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
	bool copyState = false;
	addComputation(std::make_shared<PreUpdate>(copyState));
	addComputation(std::make_shared<FreeMotion>(copyState));
	addComputation(std::make_shared<UpdateCollisionRepresentations>(copyState));
	addComputation(std::make_shared<DcdCollision>(copyState));
	addComputation(std::make_shared<ContactConstraintGeneration>(copyState));
	addComputation(std::make_shared<BuildMlcp>(copyState));
	addComputation(std::make_shared<SolveMlcp>(copyState));
	addComputation(std::make_shared<PushResults>(copyState));
	addComputation(std::make_shared<UpdateCollisionRepresentations>(copyState));
	addComputation(std::make_shared<PostUpdate>(copyState));

	return true;
}

void PhysicsManager::addComputation(std::shared_ptr<SurgSim::Physics::Computation> computation)
{
	m_computations.push_back(computation);
}

bool PhysicsManager::doStartUp()
{
	return true;
}

void PhysicsManager::getFinalState(SurgSim::Physics::PhysicsManagerState* s) const
{
	m_finalState.get(s);
}

std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>>::iterator PhysicsManager::findExcludedCollisionPair(
	std::shared_ptr<SurgSim::Collision::Representation> representation1,
	std::shared_ptr<SurgSim::Collision::Representation> representation2)
{
	return std::find_if(m_excludedCollisionPairs.begin(), m_excludedCollisionPairs.end(),
		[&representation1, &representation2] (const std::shared_ptr<SurgSim::Collision::CollisionPair>&pair)
		{
			return (pair->getFirst() == representation1 && pair->getSecond() == representation2)
				|| (pair->getFirst() == representation2 && pair->getSecond() == representation1);
		});
}

void PhysicsManager::addExcludedCollisionPair(std::shared_ptr<SurgSim::Collision::Representation> representation1,
											  std::shared_ptr<SurgSim::Collision::Representation> representation2)
{
	boost::mutex::scoped_lock lock(m_excludedCollisionPairMutex);

	if (findExcludedCollisionPair(representation1, representation2) == m_excludedCollisionPairs.end())
	{
		m_excludedCollisionPairs.push_back(
			std::make_shared<SurgSim::Collision::CollisionPair>(representation1, representation2));
	}
}

void PhysicsManager::removeExcludedCollisionPair(std::shared_ptr<SurgSim::Collision::Representation> representation1,
												 std::shared_ptr<SurgSim::Collision::Representation> representation2)
{
	boost::mutex::scoped_lock lock(m_excludedCollisionPairMutex);

	auto candidatePair = findExcludedCollisionPair(representation1, representation2);

	if (candidatePair != m_excludedCollisionPairs.end())
	{
		m_excludedCollisionPairs.erase(candidatePair);
	}
}

bool PhysicsManager::executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	std::shared_ptr<Representation> representation = tryAddComponent(component, &m_representations);
	std::shared_ptr<SurgSim::Collision::Representation> collisionRep =
		tryAddComponent(component, &m_collisionRepresentations);
	std::shared_ptr<ConstraintComponent> constraintComponent = tryAddComponent(component, &m_constraintComponents);
	return representation != nullptr || collisionRep != nullptr || constraintComponent != nullptr;
}

bool PhysicsManager::executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	bool removed1 = tryRemoveComponent(component, &m_representations);
	bool removed2 = tryRemoveComponent(component, &m_collisionRepresentations);
	bool removed3 = tryRemoveComponent(component, &m_constraintComponents);
	return removed1 || removed2 || removed3;
}

bool PhysicsManager::doUpdate(double dt)
{
	// Add all components that came in before the last update
	processComponents();

	auto state = std::make_shared<PhysicsManagerState>();
	std::list<std::shared_ptr<PhysicsManagerState>> stateList(1, state);
	state->setRepresentations(m_representations);
	state->setCollisionRepresentations(m_collisionRepresentations);
	state->setConstraintComponents(m_constraintComponents);

	{
		boost::mutex::scoped_lock lock(m_excludedCollisionPairMutex);
		state->setExcludedCollisionPairs(m_excludedCollisionPairs);
	}

	for (const auto& computation : m_computations)
	{
		stateList.push_back(computation->update(dt, stateList.back()));
	}

	m_finalState.set(*(stateList.back()));

	// Add/remove all components that were treated by the behaviors
	processBehaviors(dt);

	return true;
}

}; // Physics
}; // SurgSim

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

	processBehaviors(dt);

	std::list<std::shared_ptr<PhysicsManagerState>> stateList;
	std::shared_ptr<PhysicsManagerState> state = std::make_shared<PhysicsManagerState>();
	stateList.push_back(state);
	state->setRepresentations(m_representations);
	state->setCollisionRepresentations(m_collisionRepresentations);
	state->setConstraintComponents(m_constraintComponents);

	{
		boost::mutex::scoped_lock lock(m_excludedCollisionPairMutex);
		state->setExcludedCollisionPairs(m_excludedCollisionPairs);
	}

	stateList.push_back(m_preUpdateStep->update(dt, stateList.back()));
	stateList.push_back(m_freeMotionStep->update(dt, stateList.back()));
	stateList.push_back(m_updateCollisionRepresentationsStep->update(dt, stateList.back()));
	stateList.push_back(m_dcdCollisionStep->update(dt, stateList.back()));
	stateList.push_back(m_constraintGenerationStep->update(dt, stateList.back()));
	stateList.push_back(m_buildMlcpStep->update(dt, stateList.back()));
	stateList.push_back(m_solveMlcpStep->update(dt, stateList.back()));
	stateList.push_back(m_pushResultsStep->update(dt, stateList.back()));
	stateList.push_back(m_updateCollisionRepresentationsStep->update(dt, stateList.back()));
	stateList.push_back(m_postUpdateStep->update(dt, stateList.back()));

	m_finalState.set(*(stateList.back()));

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
	m_updateCollisionRepresentationsStep.reset(new UpdateCollisionRepresentations(copyState));

	if (m_maxIterations.hasValue())
	{
		m_solveMlcpStep->setMaxIterations(m_maxIterations.getValue());
	}
	if (m_precision.hasValue())
	{
		m_solveMlcpStep->setPrecision(m_precision.getValue());
	}
	if (m_contactTolerance.hasValue())
	{
		m_solveMlcpStep->setContactTolerance(m_contactTolerance.getValue());
	}
}

void PhysicsManager::setMaxIterations(int maxIterations)
{
	m_maxIterations.setValue(maxIterations);
}

int PhysicsManager::getMaxIterations() const
{
	int result;
	if (m_maxIterations.hasValue())
	{
		result = m_maxIterations.getValue();
	}
	else
	{
		SolveMlcp solver;
		result = solver.getMaxIterations();
	}
	return result;
}

void PhysicsManager::setPrecision(double precision)
{
	m_precision.setValue(precision);
}

double PhysicsManager::getPrecision() const
{
	double result;
	if (m_precision.hasValue())
	{
		result = m_precision.getValue();
	}
	else
	{
		SolveMlcp solver;
		result = solver.getPrecision();
	}
	return result;
}

void PhysicsManager::setContactTolerance(double contactTolerance)
{
	m_contactTolerance.setValue(contactTolerance);
}

double PhysicsManager::getContactTolerance() const
{
	double result;
	if (m_contactTolerance.hasValue())
	{
		result = m_contactTolerance.getValue();
	}
	else
	{
		SolveMlcp solver;
		result = solver.getContactTolerance();
	}
	return result;
}


}; // Physics
}; // SurgSim

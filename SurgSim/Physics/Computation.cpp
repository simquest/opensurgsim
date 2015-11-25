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

#include "SurgSim/Physics/Computation.h"

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

namespace SurgSim
{
namespace Physics
{

Computation::Computation(bool doCopyState) : m_copyState(doCopyState)
{

}

Computation::~Computation()
{

}

std::shared_ptr<PhysicsManagerState> Computation::update(double dt, const std::shared_ptr<PhysicsManagerState>& state)
{
	m_timer.beginFrame();
	auto newState = doUpdate(dt, preparePhysicsState(state));
	m_timer.endFrame();
	return newState;
}

void Computation::setDoCopyState(bool val)
{
	m_copyState = val;
}

bool Computation::isCopyingState()
{
	return m_copyState;
}

Framework::Timer& Computation::getTimer()
{
	return m_timer;
}

std::shared_ptr<PhysicsManagerState> Computation::preparePhysicsState(const std::shared_ptr<PhysicsManagerState>& state)
{
	auto isInactive = [](std::shared_ptr<Framework::Component> component)
	{
		return !component->isActive();
	};

	// Compile the list of active representations and set it on the state.
	auto representations = state->getRepresentations();
	representations.erase(std::remove_if(representations.begin(), representations.end(), isInactive),
			representations.end());
	state->setActiveRepresentations(representations);

	// Compile the list of active collision representations and set it on the state.
	auto collisionRepresentations = state->getCollisionRepresentations();
	collisionRepresentations.erase(
			std::remove_if(collisionRepresentations.begin(), collisionRepresentations.end(), isInactive),
			collisionRepresentations.end());
	state->setActiveCollisionRepresentations(collisionRepresentations);

	// Compile the list of active particle representations and set it on the state.
	auto particleRepresentations = state->getParticleRepresentations();
	particleRepresentations.erase(
			std::remove_if(particleRepresentations.begin(), particleRepresentations.end(), isInactive),
			particleRepresentations.end());
	state->setActiveParticleRepresentations(particleRepresentations);

	// Compile the list of active constraints and set it on the state.
	std::vector<std::shared_ptr<Constraint>> activeConstraints;
	size_t size = 0;
	int constraintTypeEnd = static_cast<int>(CONSTRAINT_GROUP_TYPE_COUNT);
	for (int constraintType = 0 ; constraintType < constraintTypeEnd ; constraintType++)
	{
		size += state->getConstraintGroup(constraintType).size();
	}
	activeConstraints.reserve(size);

	for (int constraintType = 0 ; constraintType < constraintTypeEnd ; constraintType++)
	{
		auto constraints = state->getConstraintGroup(constraintType);
		for (auto it = constraints.begin(); it != constraints.end(); it++)
		{
			if ((*it)->isActive())
			{
				activeConstraints.push_back(*it);
			}
		}
	}
	state->setActiveConstraints(activeConstraints);

	if (m_copyState)
	{
		return std::move(std::make_shared<PhysicsManagerState>(*state));
	}
	else
	{
		return std::move(state);
	}
}

}; // Physics
}; // SurgSim

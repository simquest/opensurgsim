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
	return std::move(doUpdate(dt,std::move(preparePhysicsState(state))));
}

void Computation::setDoCopyState(bool val)
{
	m_copyState = val;
}

bool Computation::isCopyingState()
{
	return m_copyState;
}

std::shared_ptr<PhysicsManagerState> Computation::preparePhysicsState(const std::shared_ptr<PhysicsManagerState>& state)
{
	// Compile the list of active representations and set it on the state.
	std::vector<std::shared_ptr<Representation>> activeRepresentations;
	auto& representations = state->getRepresentations();
	activeRepresentations.reserve(representations.size());
	for (auto& representation : representations)
	{
		if (representation->isActive())
		{
			activeRepresentations.push_back(representation);
		}
	}
	state->setActiveRepresentations(activeRepresentations);

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

	// Compile the list of active collision representations and set it on the state.
	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> activeCollisionRepresentations;
	const auto& collisionRepresentations = state->getCollisionRepresentations();
	activeCollisionRepresentations.reserve(collisionRepresentations.size());
	for (const auto& collisionRepresentation : collisionRepresentations)
	{
		if (collisionRepresentation->isActive())
		{
			activeCollisionRepresentations.push_back(collisionRepresentation);
		}
	}
	state->setActiveCollisionRepresentations(activeCollisionRepresentations);

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

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

/// \file PhysicsManagerStateTestCommon.h
/// Common utility functions for for PhysicsManagerState tests

#ifndef SURGSIM_PHYSICS_UNITTESTS_PHYSICSMANAGERSTATETESTCOMMON_H
#define SURGSIM_PHYSICS_UNITTESTS_PHYSICSMANAGERSTATETESTCOMMON_H

#include <unordered_map>
#include <vector>

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Physics/BuildMlcp.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

using SurgSim::Physics::Constraint;
using SurgSim::Physics::PhysicsManagerState;
using SurgSim::Physics::Representation;

namespace
{
void filterActiveRepresentations(std::shared_ptr<PhysicsManagerState> state)
{
	std::vector<std::shared_ptr<Representation>> activeRepresentations;
	auto representations = state->getRepresentations();
	activeRepresentations.reserve(representations.size());
	for (auto it = representations.begin(); it != representations.end(); ++it)
	{
		if ((*it)->isActive())
		{
			activeRepresentations.push_back(*it);
		}
	}
	state->setActiveRepresentations(activeRepresentations);
}

void filterActiveConstraints(std::shared_ptr<PhysicsManagerState> state)
{
	std::vector<std::shared_ptr<Constraint>> activeConstraints;
	size_t size = 0;
	int constraintTypeEnd = static_cast<int>(SurgSim::Physics::CONSTRAINT_GROUP_TYPE_COUNT);
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
}

void updateRepresentationsMapping(std::shared_ptr<PhysicsManagerState> state)
{
	SurgSim::Physics::MlcpMapping<Representation> representationsMapping;

	ptrdiff_t index = 0;
	filterActiveRepresentations(state); // This is usually done in preparePhysicsState
	auto const activeRepresentations = state->getActiveRepresentations();
	for (auto it = activeRepresentations.begin(); it != activeRepresentations.end(); it++)
	{
		representationsMapping.setValue((*it).get(), index);
		index += (*it)->getNumDof();
	}
	state->setRepresentationsMapping(representationsMapping);
}

void updateConstraintsMapping(std::shared_ptr<PhysicsManagerState> state)
{
	SurgSim::Physics::MlcpMapping<Constraint> constraintsMapping;

	ptrdiff_t index = 0;
	filterActiveConstraints(state); // This is usually done in preparePhysicsState
	auto const activeConstraints = state->getActiveConstraints();
	for (auto it = activeConstraints.begin(); it != activeConstraints.end(); it++)
	{
		constraintsMapping.setValue((*it).get(), index);
		index += (*it)->getNumDof();
	}
	state->setConstraintsMapping(constraintsMapping);
}
}

#endif // SURGSIM_PHYSICS_UNITTESTS_PHYSICSMANAGERSTATETESTCOMMON_H
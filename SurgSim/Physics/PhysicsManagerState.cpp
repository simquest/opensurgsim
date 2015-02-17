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

#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintComponent.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/Representation.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"

namespace SurgSim
{
namespace Physics
{

PhysicsManagerState::PhysicsManagerState()
{

}

PhysicsManagerState::~PhysicsManagerState()
{

}

void PhysicsManagerState::setRepresentations(const std::vector<std::shared_ptr<Representation>>& val)
{
	m_representations = val;

	m_collisionsToPhysicsMap.clear();
	for (auto it = m_representations.begin(); it != m_representations.end(); it++)
	{
		if ((*it)->isActive())
		{
			auto collision = (*it)->getCollisionRepresentation();
			if (collision != nullptr)
			{
				m_collisionsToPhysicsMap[collision] = (*it);
			}
		}
	}
}

const std::vector<std::shared_ptr<Representation>>& PhysicsManagerState::getRepresentations()
{
	return m_representations;
}

void PhysicsManagerState::setActiveRepresentations(
	const std::vector<std::shared_ptr<Representation>>& activeRepresentations)
{
	m_activeRepresentations = activeRepresentations;
}

const std::vector<std::shared_ptr<Representation>>& PhysicsManagerState::getActiveRepresentations() const
{
	return m_activeRepresentations;
}

const std::unordered_map<
	std::shared_ptr<SurgSim::Collision::Representation>,
	std::shared_ptr<SurgSim::Physics::Representation>>&
	PhysicsManagerState::getCollisionToPhysicsMap() const
{
	return m_collisionsToPhysicsMap;
}

void PhysicsManagerState::setCollisionRepresentations(
	const std::vector<std::shared_ptr<SurgSim::Collision::Representation>>& val)
{
	m_collisionRepresentations = val;
}

const std::vector<std::shared_ptr<SurgSim::Collision::Representation>>&
	PhysicsManagerState::getCollisionRepresentations()
{
	return m_collisionRepresentations;
}

void PhysicsManagerState::setActiveCollisionRepresentations(
	const std::vector<std::shared_ptr<SurgSim::Collision::Representation>>& val)
{
	m_activeCollisionRepresentations = val;
}

const std::vector<std::shared_ptr<SurgSim::Collision::Representation>>&
	PhysicsManagerState::getActiveCollisionRepresentations()
{
	return m_activeCollisionRepresentations;
}

void PhysicsManagerState::setConstraintComponents(const std::vector<std::shared_ptr<ConstraintComponent>>& val)
{
	m_constraintComponents = val;

	std::vector<std::shared_ptr<Constraint>>& constraints = m_constraints[CONSTRAINT_GROUP_TYPE_SCENE];

	constraints.reserve(m_constraintComponents.size());
	constraints.clear();
	for (auto it = m_constraintComponents.cbegin(); it != m_constraintComponents.cend(); ++it)
	{
		if ((*it)->getConstraint()->isActive())
		{
			constraints.push_back((*it)->getConstraint());
		}
	}

	setConstraintGroup(CONSTRAINT_GROUP_TYPE_SCENE, constraints);
}

const std::vector<std::shared_ptr<ConstraintComponent>>& PhysicsManagerState::getConstraintComponents()
{
	return m_constraintComponents;
}

void PhysicsManagerState::setCollisionPairs(const std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>>& val)
{
	m_collisionPairs = val;
}

const std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>>& PhysicsManagerState::getCollisionPairs()
{
	return m_collisionPairs;
}

void PhysicsManagerState::setExcludedCollisionPairs(
	const std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>>& val)
{
	m_excludedCollisionPairs = val;
}

const std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>>&
	PhysicsManagerState::getExcludedCollisionPairs() const
{
	return m_excludedCollisionPairs;
}

void PhysicsManagerState::setConstraintGroup(
	ConstraintGroupType type,
	const std::vector<std::shared_ptr<Constraint>>& constraints)
{
	m_constraints[type] = constraints;
}

const std::vector<std::shared_ptr<Constraint>>& PhysicsManagerState::getConstraintGroup(int type) const
{
	if (m_constraints.count(type) > 0)
	{
		return m_constraints.at(type);
	}
	static std::vector<std::shared_ptr<Constraint>> emptyVector;
	return emptyVector;
}

void PhysicsManagerState::setActiveConstraints(
	const std::vector<std::shared_ptr<Constraint>>& activeConstraints)
{
	m_activeConstraints = activeConstraints;
}

const std::vector<std::shared_ptr<Constraint>>& PhysicsManagerState::getActiveConstraints() const
{
	return m_activeConstraints;
}

MlcpPhysicsProblem& PhysicsManagerState::getMlcpProblem()
{
	return m_mlcpPhysicsProblem;
}

const MlcpPhysicsProblem& PhysicsManagerState::getMlcpProblem() const
{
	return m_mlcpPhysicsProblem;
}

MlcpPhysicsSolution& PhysicsManagerState::getMlcpSolution()
{
	return m_mlcpPhysicsSolution;
}

const MlcpPhysicsSolution& PhysicsManagerState::getMlcpSolution() const
{
	return m_mlcpPhysicsSolution;
}

const MlcpMapping<Representation>& PhysicsManagerState::getRepresentationsMapping() const
{
	return m_representationsIndexMapping;
}

void PhysicsManagerState::setRepresentationsMapping(const MlcpMapping<Representation>& representationsMapping)
{
	m_representationsIndexMapping = representationsMapping;
}

const MlcpMapping<Constraint>& PhysicsManagerState::getConstraintsMapping() const
{
	return m_constraintsIndexMapping;
}

void PhysicsManagerState::setConstraintsMapping(const MlcpMapping<Constraint>& constraintsMapping)
{
	m_constraintsIndexMapping = constraintsMapping;
}

}; // Physics
}; // SurgSim

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

#ifndef SURGSIM_PHYSICS_PHYSICSMANAGERSTATE_H
#define SURGSIM_PHYSICS_PHYSICSMANAGERSTATE_H

#include <memory>
#include <vector>
#include <unordered_map>

namespace SurgSim
{
namespace Physics
{

class Representation;
class CollisionPair;
class Constraint;

enum ConstraintGroupType
{
	CONSTRAINT_GROUP_TYPE_CONTACT = 0,
	CONSTRAINT_GROUP_TYPE_COUNT
};

class PhysicsManagerState
{
public:

	/// Constructor
	PhysicsManagerState() {}
	~PhysicsManagerState() {}

	const std::vector<std::shared_ptr<Representation>>& getRepresentations() const
	{ return m_representations; }
	void setRepresentations(const std::vector<std::shared_ptr<Representation>>& val)
	{ m_representations = val; }

	const std::vector<std::shared_ptr<CollisionPair>>& getCollisionPairs() const
	{ return m_collisionPairs; }
	void setCollisionPairs(std::vector<std::shared_ptr<CollisionPair>> val)
	{ m_collisionPairs = val; }

	const std::vector<std::shared_ptr<Constraint>>& getConstraintGroup(ConstraintGroupType type)
	{
		return m_constraints[type];
	}

	void setConstraintGroup(ConstraintGroupType type, const std::vector<std::shared_ptr<Constraint>>& constraints)
	{
		m_constraints[type] = constraints;
	}
private:

	std::vector<std::shared_ptr<Representation>> m_representations;
	std::vector<std::shared_ptr<CollisionPair>> m_collisionPairs;

	std::unordered_map<ConstraintGroupType, std::vector<std::shared_ptr<Constraint>>> m_constraints;

};

}; // Physics
}; // SurgSim

#endif

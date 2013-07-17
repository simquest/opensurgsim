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

	/// Sets the representations for the state, these are the basis for all the computations.
	/// \param	val The list of representations.
	void setRepresentations(const std::vector<std::shared_ptr<Representation>>& val)
	{
		m_representations = val; 
	}

	/// Gets the representations.
	/// \return	The representations.
	const std::vector<std::shared_ptr<Representation>>& getRepresentations() const
	{
		return m_representations;
	}

	/// Sets collision pairs that should be considered, while this is not being verified the collision pairs
	/// should only be from the list of representations that are in this state.
	/// \param	val	The list of collision pairs.
	void setCollisionPairs(std::vector<std::shared_ptr<CollisionPair>> val)
	{
		m_collisionPairs = val;
	}

	/// Gets collision pairs.
	/// \return	The collision pairs.
	const std::vector<std::shared_ptr<CollisionPair>>& getCollisionPairs() const
	{
		return m_collisionPairs;
	}

	/// Sets the group of constraints to a given value, the grouping indicates what type of constraint we are dealing
	/// with.
	/// \param	type	   	The type of constraint grouping e.g. Contact Constraints.
	/// \param	constraints	The constraints.
	void setConstraintGroup(ConstraintGroupType type, const std::vector<std::shared_ptr<Constraint>>& constraints)
	{
		m_constraints[type] = constraints;
	}

	/// Gets constraint group.
	/// \param	type	The type.
	/// \return	The constraint group.
	const std::vector<std::shared_ptr<Constraint>>& getConstraintGroup(ConstraintGroupType type)
	{
		return m_constraints[type];
	}

private:

	///@{
	/// Local state data structures, please note that the physics state may get copied, these data structures
	/// should copy their contents on copy. With the caveat that objects contained within those structures might
	/// not get copied themselves.
	/// The local list of representations
	std::vector<std::shared_ptr<Representation>> m_representations;

	/// The local list of collision pairs 
	std::vector<std::shared_ptr<CollisionPair>> m_collisionPairs;
	
	/// The local map of constraints
	std::unordered_map<int, std::vector<std::shared_ptr<Constraint>>> m_constraints;
	///@}
};

}; // Physics
}; // SurgSim

#endif

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

#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/MlcpMapping.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"
#include "SurgSim/Physics/MlcpPhysicsSolution.h"
#include "SurgSim/Physics/Representation.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"

namespace SurgSim
{
namespace Physics
{

enum ConstraintGroupType
{
	CONSTRAINT_GROUP_TYPE_CONTACT = 0,
	CONSTRAINT_GROUP_TYPE_COUNT
};

class PhysicsManagerState
{
public:
	/// Constructor
	PhysicsManagerState();
	~PhysicsManagerState();

	/// Sets the physics representations for the state, these are the basis for all the computations.
	/// \param	val The list of representations.
	void setRepresentations(const std::vector<std::shared_ptr<Representation>>& val);

	/// Gets the physics representations.
	/// \return	The physics representations that are known to the state.
	const std::vector<std::shared_ptr<Representation>>& getRepresentations();

	/// Sets the collision representations for the state.
	/// \param val collection of all collision representations.
	void setCollisionRepresentations(const std::vector<std::shared_ptr<SurgSim::Collision::Representation>>& val);

	/// Gets the collision representations.
	/// \return The collision representations that are known to the state.
	const std::vector<std::shared_ptr<SurgSim::Collision::Representation>>& getCollisionRepresentations();

	/// \return A map that associates collision representations with physicsre presentations where
	///         map[physicsRep->getCollisionRepresentation] = physicsRep
	const std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
		  std::shared_ptr<SurgSim::Physics::Representation>>& getCollisionToPhysicsMap() const;

	/// Sets collision pairs that should be considered, while this is not being verified the collision pairs
	/// should only be from the list of representations that are in this state.
	/// \param	val	The list of collision pairs.
	void setCollisionPairs(std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>> val);

	/// Gets collision pairs.
	/// \return	The collision pairs.
	const std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>>& getCollisionPairs();

	/// Sets the group of constraints to a given value, the grouping indicates what type of constraint we are dealing
	/// with.
	/// \param	type	   	The type of constraint grouping e.g. Contact Constraints.
	/// \param	constraints	The constraints.
	void setConstraintGroup(ConstraintGroupType type, const std::vector<std::shared_ptr<Constraint>>& constraints);

	/// Gets constraint group.
	/// \param	type	The type.
	/// \return	The constraint group.
	const std::vector<std::shared_ptr<Constraint>>& getConstraintGroup(int type) const;

	/// Gets the Mlcp problem
	/// \return	The Mlcp problem for this physics manager state (read/write access).
	MlcpPhysicsProblem& getMlcpProblem();

	/// Gets the Mlcp problem
	/// \return	The Mlcp problem for this physics manager state (const).
	const MlcpPhysicsProblem& getMlcpProblem() const;

	/// Gets the Mlcp solution
	/// \return	The Mlcp solution for this physics manager state (read/write access).
	MlcpPhysicsSolution& getMlcpSolution();

	/// Gets the Mlcp solution
	/// \return	The Mlcp solution for this physics manager state (const).
	const MlcpPhysicsSolution& getMlcpSolution() const;

	/// Gets the representations mapping
	/// \return	The representations mapping (mapping between the representation and the mlcp)
	/// Each representation has an index in the mlcp. This mapping is about this index.
	const MlcpMapping<Representation>& getRepresentationsMapping() const;

	/// Gets the constraints mapping
	/// \return	The constraints mapping (mapping between the constraints and the mlcp)
	/// Each constraint has an index in the mlcp. This mapping is about this index.
	const MlcpMapping<Constraint>& getConstraintsMapping() const;

private:

	///@{
	/// Local state data structures, please note that the physics state may get copied, these data structures
	/// should copy their contents on copy. With the caveat that objects contained within those structures might
	/// not get copied themselves.
	/// The local list of representations
	std::vector<std::shared_ptr<Representation>> m_representations;

	/// List of all the collision representations know to the state
	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> m_collisionRepresentations;

	/// Mapping of collision representations to their respective physics representation.
	std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
		std::shared_ptr<SurgSim::Physics::Representation>> m_collisionsToPhysicsMap;

	/// The local list of collision pairs.
	std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>> m_collisionPairs;

	/// The local map of constraints.
	std::unordered_map<int, std::vector<std::shared_ptr<Constraint>>> m_constraints;

	/// Representation mapping
	MlcpMapping<Representation> m_representationsIndexMapping;

	/// Constraints mapping
	MlcpMapping<Constraint> m_constraintsIndexMapping;

	///@}
	/// Mlcp problem for this Physics Manager State
	MlcpPhysicsProblem m_mlcpPhysicsProblem;

	/// Mlcp solution for this Physics Manager State
	MlcpPhysicsSolution m_mlcpPhysicsSolution;
};

}; // Physics
}; // SurgSim

#endif

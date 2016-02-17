// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_SLIDINGCONSTRAINT_H
#define SURGSIM_PHYSICS_SLIDINGCONSTRAINT_H

#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/SlidingConstraintData.h"

namespace SurgSim
{

namespace Physics
{

/// Sliding constraint between two physics representations.
/// The sliding direction vector (direction + origin) needs to be updated each physics time step. This is done by
/// transforming the direction and origin into the local coordinates of the physics representations and recalculating
/// them every time step.
class SlidingConstraint : public Constraint
{
public:
	/// Sets all the values for this constraints, does validation on the parameters and will throw if something
	/// is wrong with the constraint.
	/// \param constraintType The constraint type.
	/// \param data The data for this constraint.
	/// \param representation0, representation1 Both representations in this constraint.
	/// \param location0, location1 Both locations of the representations involved in this constraint.
	/// \param slidingDirection The direction of sliding.
	SlidingConstraint(
		ConstraintType constraintType,
		std::shared_ptr<ConstraintData> data,
		std::shared_ptr<Representation> representation0,
		const SurgSim::DataStructures::Location& location0,
		std::shared_ptr<Representation> representation1,
		const SurgSim::DataStructures::Location& location1,
		const Math::Vector3d& slidingDirection);

	/// Destructor
	virtual ~SlidingConstraint();

private:
	void doBuild(double dt,
		const ConstraintData& data,
		MlcpPhysicsProblem* mlcpPhysicsProblem,
		size_t indexOfRepresentation0,
		size_t indexOfRepresentation1,
		size_t indexOfConstraint) override;

	/// The sliding constraint data.
	std::shared_ptr<SlidingConstraintData> m_slidingConstraintData;

	/// The end of the sliding direction, local to the representation1.
	Math::Vector3d m_directionEnd;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_SLIDINGCONSTRAINT_H

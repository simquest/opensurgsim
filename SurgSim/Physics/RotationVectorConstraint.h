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

#ifndef SURGSIM_PHYSICS_ROTATIONVECTORCONSTRAINT_H
#define SURGSIM_PHYSICS_ROTATIONVECTORCONSTRAINT_H

#include "SurgSim/Physics/Constraint.h"

namespace SurgSim
{

namespace Physics
{

/// Specific class for rotation vector constraints.
/// It handles the specificity of the rotation vector in the constraint calculation.
class RotationVectorConstraint: public Constraint
{
public:
	RotationVectorConstraint(
		ConstraintType constraintType,
		std::shared_ptr<ConstraintData> data,
		std::shared_ptr<Representation> representation0,
		const SurgSim::DataStructures::Location& location0,
		std::shared_ptr<Representation> representation1,
		const SurgSim::DataStructures::Location& location1);

	/// Destructor
	virtual ~RotationVectorConstraint();

protected:
	/// \note The rotation vector violation being calculated based on a quaternion interpolation (slerp), and this
	/// type of interpolation being highly non-linear, the classical way of using the implementation one after the
	/// other one won't work.
	/// Therefore, this method temporarily uses the vector mlcpPhysicsProblem->b to retrieve both representation's
	/// rotation vectors, then calculate the proper slerp and set the violation back in mlcpPhysicsProblem->b
	void doBuild(double dt,
		const ConstraintData& data,
		MlcpPhysicsProblem* mlcpPhysicsProblem,
		size_t indexOfRepresentation0,
		size_t indexOfRepresentation1,
		size_t indexOfConstraint) override;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_ROTATIONVECTORCONSTRAINT_H

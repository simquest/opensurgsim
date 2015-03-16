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

#ifndef SURGSIM_PHYSICS_MASSSPRINGREPRESENTATIONCONTACT_H
#define SURGSIM_PHYSICS_MASSSPRINGREPRESENTATIONCONTACT_H

#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/Localization.h"

namespace SurgSim
{

namespace Physics
{

/// MassSpring frictionless contact implementation.
///
/// MassSpringRepresentationContact implements the frictionless contact constraint for the MassSpringRepresentation,
/// which prevents nodes from passing through a surface.  See MassSpringRepresentationContact::doBuild for more
/// information.
class MassSpringRepresentationContact : public ConstraintImplementation
{
public:
	/// Constructor
	MassSpringRepresentationContact();

	/// Destructor
	virtual ~MassSpringRepresentationContact();

	/// Gets the Mixed Linear Complementarity Problem constraint type for this ConstraintImplementation
	/// \return SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT
	SurgSim::Math::MlcpConstraintType getMlcpConstraintType() const override;

private:
	/// Gets the number of degrees of freedom for a frictionless contact.
	/// \return 1, as a frictionless contact only has 1 equation of constraint (along the normal direction).
	size_t doGetNumDof() const override;

	/// Adds a mass-spring frictionless contact constraint to an MlcpPhysicsProblem.
	/// \param dt The time step.
	/// \param data [ContactConstraintData] Plane defining the constraint.
	/// \param localization [MassSpringRepresentationLocalization] Location and Representation to be constrained.
	/// \param [in, out] mlcp The Mixed LCP physics problem to fill up.
	/// \param indexOfRepresentation The index of the representation (associated to this implementation) in the mlcp.
	/// \param indexOfConstraint The index of the constraint in the mlcp.
	/// \param sign The sign of this implementation in the constraint (positive or negative side).
	void doBuild(double dt,
		const ConstraintData& data,
		const std::shared_ptr<Localization>& localization,
		MlcpPhysicsProblem* mlcp,
		size_t indexOfRepresentation,
		size_t indexOfConstraint,
		ConstraintSideSign sign) override;

};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_MASSSPRINGREPRESENTATIONCONTACT_H

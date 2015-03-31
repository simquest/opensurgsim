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

#ifndef SURGSIM_PHYSICS_FEM3DREPRESENTATIONCONSTRAINT2FEM3DPOINTS_H
#define SURGSIM_PHYSICS_FEM3DREPRESENTATIONCONSTRAINT2FEM3DPOINTS_H

#include "SurgSim/Physics/ConstraintImplementation.h"

namespace SurgSim
{

namespace Physics
{

/// Fem3DRepresentation distance constraint between 2 points of the same Fem3D
///
/// This constraint belongs to the family of bilateral1D constraints enforcing equality between 1 dof (the distance).
class Fem3DRepresentationConstraint2Fem3DPoints : public ConstraintImplementation
{
public:
	/// Constructor
	Fem3DRepresentationConstraint2Fem3DPoints();

	/// Destructor
	virtual ~Fem3DRepresentationConstraint2Fem3DPoints();

	/// Gets the Mixed Linear Complementarity Problem constraint type for this ConstraintImplementation
	/// \return The MLCP constraint type corresponding to this constraint implementation
	SurgSim::Math::MlcpConstraintType getMlcpConstraintType() const override;

private:
	/// Gets the number of degree of freedom.
	/// \return 3 A bilateral 3d constraint enforces equality in the x, y, and z dimensions between 2 vectors.
	size_t doGetNumDof() const override;

	/// Builds the subset of an Mlcp physics problem associated to this implementation.
	/// \param dt The time step.
	/// \param data The data associated to the constraint.
	/// \param localization The localization for the representation.
	/// \param [in, out] mlcp The Mixed LCP physics problem to fill up.
	/// \param indexOfRepresentation The index of the representation (associated to this implementation) in the mlcp.
	/// \param indexOfConstraint The index of the constraint in the mlcp.
	/// \param sign The sign of this implementation in the constraint (positive or negative side).
	/// \note Empty for a Fixed Representation
	void doBuild(double dt,
				 const ConstraintData& data,
				 const std::shared_ptr<Localization>& localization,
				 MlcpPhysicsProblem* mlcp,
				 size_t indexOfRepresentation,
				 size_t indexOfConstraint,
				 ConstraintSideSign sign) override;

	//SurgSim::Math::Vector3d numericalEvaluationRotVectorJacobian(std::shared_ptr<Fem3DRepresentation> fem3d, FemElement* femElement,
	//	SurgSim::Math::Quaterniond originalStaplerRotation,
	//	SurgSim::Math::Quaterniond originalFemElementOrientation,
	//	SurgSim::Math::Quaterniond currentFemElementOrientation,
	//	size_t dofId, double epsilon);

	//SurgSim::Math::Vector3d numericalEvaluationRotVector(std::shared_ptr<Fem3DRepresentation> fem3d, FemElement* femElement,
	//	SurgSim::Math::Quaterniond originalStaplerRotation,
	//	SurgSim::Math::Quaterniond originalFemElementOrientation,
	//	SurgSim::Math::Quaterniond currentFemElementOrientation);

	//std::shared_ptr<SurgSim::Math::OdeState> m_tmpFem3DState;
};

}; // namespace Physics

}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DREPRESENTATIONCONSTRAINT2FEM3DPOINTS_H

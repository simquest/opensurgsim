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

#ifndef SURGSIM_PHYSICS_MLCPPHYSICSPROBLEM_H
#define SURGSIM_PHYSICS_MLCPPHYSICSPROBLEM_H

#include "SurgSim/Math/MlcpProblem.h"
#include "SurgSim/Math/SparseMatrix.h"

namespace SurgSim
{
namespace Physics
{

/// A description of a physical mixed LCP system to be solved.
///

/// This extends \ref SurgSim::Math::MlcpProblem "the pure mathematical MLCP problem" by storing the intermediate
/// matrices \ref H and \ref CHt that are necessary to physically interpret the solution.
///
/// Note that the matrix \f$\mathbf{A}\f$ used in the MlcpProblem is computed in the physical problem as
/// \f$\mathbf{H\;C\;H^T}\f$, where \f$\mathbf{C}\f$ is the compliance matrix.  For contact constraints, \f$b\f$ is
/// the initial signed displacements between the colliding representations, \f$b_i \lt 0\f$ when the representations
/// interpenetrate, \f$x\f$ is the forces to apply at each contact to prevent penetration, and \f$c\f$ is the signed
/// displacements after the forces are applied.
/// \note The solution to the MLCP will only address the constraints that were provided, and application of \f$x\f$ to
/// the representations in the scene may cause new collisions for constraints that were not originally incorporated in
/// the MLCP.
///
/// \sa SurgSim::Math::MlcpProblem
struct MlcpPhysicsProblem : public SurgSim::Math::MlcpProblem
{
	/// Destructor
	~MlcpPhysicsProblem() override;

	/// The matrix \f$\mathbf{H}\f$, which is a matrix of size \f$c\times n\f$ that converts from
	/// the \f$n\f$ degrees of freedom in the system (i.e., the sum of all the DOF over all the representations in the
	/// scene), to the
	/// \f$c\f$ degrees of freedom summed over all the constraints being applied to the system.
	/// It is used to convert the vector of \f$n\f$ displacements of each degree of freedom of the system to the vector
	/// of \f$c\f$ displacements of each degree of freedom of the constraints.
	/// Given a set of constraints \f$\mathbf{G}(t, \mathbf{x})\f$, then
	/// \f$\mathbf{H} = \frac{d \mathbf{G}}{d \mathbf{x}}\f$ (i.e., the constraints' tangential space).
	Eigen::SparseMatrix<double, Eigen::RowMajor, ptrdiff_t> H;

	/// The matrix \f$\mathbf{C\;H^T}\f$, which is a matrix of size \f$n\times c\f$ that is used to convert the
	/// vector of \f$c\f$ constraint forces to the \f$n\f$ displacements of each degree of freedom of the system.
	Matrix CHt;

	/// Applies a new constraint to a specific Representation
	/// \param newSubH New constraint to be added to H
	/// \param subC Compliance matrix associated with the Representation
	/// \param indexSubC Index of the Representation's compliance matrix
	/// \param indexNewSubH Index of the new constraint within H
	/// \tparam SubCDerivedType the CRTP derived type of the passed subC matrix, which usually can be deduced
	template <typename SubCDerivedType>
	void updateConstraint(
		const Eigen::SparseVector<double>& newSubH,
		const Eigen::MatrixBase<SubCDerivedType>& subC,
		size_t indexSubC,
		size_t indexNewSubH);

	/// Resize an MlcpPhysicsProblem and set to zero.
	/// \param numDof the total degrees of freedom.
	/// \param numConstraintDof the total constrained degrees of freedom.
	/// \param numConstraints the number of constraints.
	void setZero(size_t numDof, size_t numConstraintDof, size_t numConstraints) override;

	/// Initialize an MlcpPhysicsProblem with zero values.
	/// \param numDof the total degrees of freedom for the MlcpPhysicsProblem to be constructed.
	/// \param numConstraintDof the total constrained degrees of freedom for the MlcpPhysicsProblem to be constructed.
	/// \param numConstraints the number of constraints for the MlcpPhysicsProblem to be constructed.
	/// \return An MlcpPhysicsProblem appropriately sized and initialized to zero.
	static MlcpPhysicsProblem Zero(size_t numDof, size_t numConstraintDof, size_t numConstraints);
};

};  // namespace Physics
};  // namespace SurgSim

#include "SurgSim/Physics/MlcpPhysicsProblem-inl.h"

#endif  // SURGSIM_PHYSICS_MLCPPHYSICSPROBLEM_H

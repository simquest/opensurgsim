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
#include <Eigen/SparseCore>

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
///
/// \sa SurgSim::Math::MlcpProblem
struct MlcpPhysicsProblem : public SurgSim::Math::MlcpProblem
{
	/// The matrix \f$\mathbf{H}\f$, which is a matrix of size\f$c\times n\f$ that converts from
	/// the \f$n\f$ degrees of freedom in the system (i.e., the sum of all the DOF over all the representations in the
	/// scene), to the 
	/// \f$c\f$ degrees of freedom summed over all the constraints being applied to the system.
	/// It is used to convert the vector of \f$n\f$ displacements of each degree of freedom of the system to the vector
	/// of \f$c\f$ displacements of each degree of freedom of the constraints.
	/// \f$\mathbf{H}\f$ is the constraints' tangential space.
	Matrix H;

	/// The matrix \f$\mathbf{C\;H^T}\f$, which is a matrix of size \f$n\times c\f$ that is used to convert the
	/// vector of \f$c\f$ constraint forces to the \f$n\f$ displacements of each degree of freedom of the system.
	Matrix CHt;

	/// Applies a new constraint to a specific Representation
	/// \param newSubH New constraint to be added to H
	/// \param subC Compliance matrix associated with the Representation
	/// \param indexSubC Index of the Representation's compliance matrix
	/// \param indexNewSubH Index of the new constraint within H
	void updateConstraint(
		const Eigen::SparseVector<double> &newSubH,
		const Eigen::MatrixXd &subC,
		size_t indexSubC,
		size_t indexNewSubH);
};

};  // namespace Physics
};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_MLCPPHYSICSPROBLEM_H

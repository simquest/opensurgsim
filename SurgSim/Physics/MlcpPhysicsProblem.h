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

#ifndef SURGSIM_MATH_MLCPPHYSICSPROBLEM_H
#define SURGSIM_MATH_MLCPPHYSICSPROBLEM_H

#include <SurgSim/Math/MlcpProblem.h>

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
/// \f$\mathbf{H\;C\;H^T}\f$.
///
/// \sa SurgSim::Math::MlcpProblem
struct MlcpPhysicsProblem : public SurgSim::Math::MlcpProblem
{
	/// The matrix \f$\mathbf{H}\f$, which is a constraints matrix of size\f$c\times n\f$ that is used to describe
	/// the correspondence between the \f$c\f$ constraints being applied to the system, and the \f$n\f$ total
	/// system degrees of freedom.
	Eigen::MatrixXd H;

	/// The matrix \f$\mathbf{C\;H^T}\f$, which is a matrix of size \f$n\times c\f$ that is used to convert the
	/// vector of \f$c\f$ constraint forces to the \f$n\f$ displacements of each degree of freedom of the system.
	Eigen::MatrixXd CHt;
};

};  // namespace Physics
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPPHYSICSPROBLEM_H

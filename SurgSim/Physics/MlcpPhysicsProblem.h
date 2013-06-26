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

/// A description of a physics MLCP (mixed linear complementarity problem, or mixed LCP) system to be solved.
///
/// It extends the typical mathematrical MLCP problem by storing intermediate matrices:
/// H(c x n) The constraints matrix containing c rows of constraints and n columns of global dof
/// CHt (n x c) The mapping matrix from forces (in constraint space) to displacement (in global dof space)
/// Note that with these notation, A in MlcpProblem is H.CHt
struct MlcpPhysicsProblem : public SurgSim::Math::MlcpProblem
{
	/// Matrix \f$\mathbf{H}\f$ used to describe the constraint wrt dof
	Eigen::MatrixXd H;
	/// Matrix \f$\mathbf{C.H^t}\f$ used to map forces (constraint space) to displacement (dof space)
	Eigen::MatrixXd CHt;
};

};  // namespace Physics
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPPHYSICSPROBLEM_H

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

#ifndef SURGSIM_MATH_MLCPSOLUTION_H
#define SURGSIM_MATH_MLCPSOLUTION_H

#include <Eigen/Core>
#include "SurgSim/Math/MlcpConstraintType.h"

namespace SurgSim
{
namespace Math
{

/// The description of a solution to a \ref MlcpProblem "mixed linear complementarity problem".
///
/// The solution consists of the vector \f$x\f$ and various diagnostic parameters.
/// If \f$c = \mathbf{A}x + b\f$ is also needed, it can be computed by the caller.
///
/// \sa SurgSim::Physics::MlcpPhysicsSolution, MlcpProblem, MlcpSolver
struct MlcpSolution
{
	typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;

	/// Vector \f$x\f$ specifying a solution to the specified mixed LCP problem.
	Vector x;

	/// The number of iterations performed.
	size_t numIterations;
	/// True if the final value of the convergence criteria is valid.
	bool validConvergence;
	/// True if the final solution satisfies the Signorini conditions.
	bool validSignorini;
	/// The final value of the convergence criteria.
	double convergenceCriteria;
	/// The initial value of the convergence criteria, before the solver has done anything.
	double initialConvergenceCriteria;
	/// The final value of the convergence criteria for each of the constraint types.
	double constraintConvergenceCriteria[MLCP_NUM_CONSTRAINT_TYPES];
	/// The initial value of the convergence criteria for each of the constraint types.
	double initialConstraintConvergenceCriteria[MLCP_NUM_CONSTRAINT_TYPES];

	// NB: We let the compiler generate the default code for the constructor, copy constructor and copy assignment,
	// because we currently sometimes need to copy the solution (although we ought to minimize this).
	// The C++11-ish way to indicate that explicitly would be to write code like this:
	//     MlcpProblem() = default;
	//     MlcpProblem(const MlcpProblem& other) = default;
	//     MlcpProblem& operator= (const MlcpProblem& other) = default;
	// but I haven't yet tested that this works correctly on VS 2010, so I'm just putting in the comment.
	// We may also want to add move construction and move assignment.  --advornik 2013-06-24
};

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPSOLUTION_H

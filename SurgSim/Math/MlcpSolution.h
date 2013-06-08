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

namespace SurgSim
{
namespace Math
{

/// The description of a solution to an MLCP problem.
///
/// The solution consists of the vector \f$x\f$ and various diagnostic parameers.
/// If \f$c\f$ is also needed, it can be computed by the caller.
///
/// \sa MlcpProblem, MlcpSolver

struct MlcpSolution
{
	Eigen::VectorXd x;

	int numIterations;
	bool validConvergence;
	bool validSignorini;
	double convergenceCriteria;
	double initialConvergenceCriteria;
	double constraintConvergenceCriteria[MLCP_NUM_CONSTRAINT_TYPES];
	double initialConstraintConvergenceCriteria[MLCP_NUM_CONSTRAINT_TYPES];

	MlcpSolution()
	{
	}

private:
	// Prevent copying and assignment
	MlcpSolution(const MlcpSolution& other);
	MlcpSolution& operator= (const MlcpSolution& other);
};

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPSOLUTION_H

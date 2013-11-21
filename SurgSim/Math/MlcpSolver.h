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

#ifndef SURGSIM_MATH_MLCPSOLVER_H
#define SURGSIM_MATH_MLCPSOLVER_H

namespace SurgSim
{
namespace Math
{

struct MlcpProblem;
struct MlcpSolution;

/// This class provides a solver interface for mixed linear complementarity problems.
///
/// \sa MlcpProblem
class MlcpSolver
{
public:
	/// Constructor.
	MlcpSolver()
	{
	}

	// Destructor.
	virtual ~MlcpSolver()
	{
	}

	/// Attempts to solve the specified MLCP problem.
	/// \param problem the MLCP problem.
	/// \param [out] solution the solution to the problem, if available.
	/// \return true if solved (in which case solution will be set to the solution); false if failed.
	virtual bool solve(const MlcpProblem& problem, MlcpSolution* solution) = 0;

private:
	/// Prevent copy construction and assignment.
	MlcpSolver(const MlcpSolver&);
	MlcpSolver& operator==(const MlcpSolver&);
};

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPSOLVER_H

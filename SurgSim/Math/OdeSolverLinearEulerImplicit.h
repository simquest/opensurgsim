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

#ifndef SURGSIM_MATH_ODESOLVERLINEAREULERIMPLICIT_H
#define SURGSIM_MATH_ODESOLVERLINEAREULERIMPLICIT_H

#include "SurgSim/Math/OdeSolverEulerImplicit.h"

namespace SurgSim
{

namespace Math
{

/// Linear Version of the Euler Implicit ode solver
/// This solver assumes that the system is linear,
/// ie that Mass, Damping, and Stiffness matrices do not change.
/// \note If the matrices are all constant, the problem to solve becomes linear,
/// \note therefore the Newton-Raphson algorithm will be exact in only 1 iteration.
/// \note Therefore this solver uses a number of maximum iteration of 1 unless overriden.
class OdeSolverLinearEulerImplicit : public OdeSolverEulerImplicit
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolverLinearEulerImplicit(OdeEquation* equation);

	void setNewtonRaphsonMaximumIteration(size_t maximumIteration) override;

	void solve(double dt, const OdeState& currentState, OdeState* newState) override;

private:
	/// The constant stiffness matrix
	Matrix m_constantK;

	/// Has the solver been initialized
	bool m_initialized;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVERLINEAREULERIMPLICIT_H

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

#ifndef SURGSIM_MATH_ODESOLVEREULERIMPLICIT_H
#define SURGSIM_MATH_ODESOLVEREULERIMPLICIT_H

#include "SurgSim/Math/OdeSolver.h"

namespace SurgSim
{

namespace Math
{

/// Euler implicit (a.k.a backward Euler) ode solver (see %OdeSolverEulerImplicit.dox for more details).
class OdeSolverEulerImplicit : public OdeSolver
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolverEulerImplicit(OdeEquation* equation);

	/// \param maximumIteration The Newton-Raphson algorithm maximum number of iterations
	virtual void setNewtonRaphsonMaximumIteration(size_t maximumIteration);

	/// \return The Newton-Raphson algorithm maximum number of iterations
	size_t getNewtonRaphsonMaximumIteration() const;

	/// \param epsilonConvergence The Newton-Raphson algorithm epsilon convergence
	void setNewtonRaphsonEpsilonConvergence(double epsilonConvergence);

	/// \return The Newton-Raphson algorithm epsilon convergence
	double getNewtonRaphsonEpsilonConvergence() const;

	void solve(double dt, const OdeState& currentState, OdeState* newState) override;

protected:
	void assembleLinearSystem(double dt, const OdeState& state, const OdeState& newState,
							  bool computeRHS = true) override;

	/// Newton-Raphson maximum number of iteration (1 => linearization)
	size_t m_maximumIteration;

	/// Newton-Raphson convergence criteria (variation of the solution over time)
	double m_epsilonConvergence;

	/// Newton-Raphson previous solution (we solve a problem to find deltaV, the variation in velocity)
	Vector m_previousSolution;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVEREULERIMPLICIT_H

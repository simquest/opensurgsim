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

/// Euler Implicit ode solver
/// \note M(x(t), v(t)).a(t) = f(t, x(t), v(t)) <BR>
/// \note This ode equation is solved as an ode of order 1 by defining the state vector y = (x v)^t: <BR>
/// \note y' = ( x' ) = ( dx/dt ) = (       v        ) <BR>
/// \note      ( v' ) = ( dv/dt ) = ( M(x, v)^{-1}.f(x, v) ) <BR>
/// \note y' = f(t, y) <BR>
/// \note Euler Implicit is also called backward Euler as it solves this integral using a backward evaluation: <BR>
/// \note y' = (y(t) - y(t-dt)) / dt <BR>
/// \note which leads to the integration scheme: <BR>
/// \note { x(t+dt) = x(t) + dt.v(t+dt) <BR>
/// \note { v(t+dt) = v(t) + dt.a(t+dt) <BR>
/// \note Euler Implicit leads to a non-linear system to solve. We use Newton-Raphson to solve this problem. <BR>
/// \note http://en.wikipedia.org/wiki/Newton%27s_method
class OdeSolverEulerImplicit : public OdeSolver
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolverEulerImplicit(OdeEquation* equation);

	virtual void setNewtonRaphsonMaximumIteration(size_t maximumIteration);

	size_t getNewtonRaphsonMaximumIteration() const;

	void setNewtonRaphsonEpsilonConvergence(double epsilonConvergence);

	double getNewtonRaphsonEpsilonConvergence() const;

	void solve(double dt, const OdeState& currentState, OdeState* newState) override;

protected:
	/// Newton-Raphson maximum number of iteration (1 => linearization)
	size_t m_maximumIteration;

	/// Newton-Raphson convergence criteria (variation of the solution over time)
	double m_epsilonConvergence;

	/// Newton-Raphson current and previous solution (we solve a problem to find deltaV, the variation in velocity)
	Vector m_deltaV, m_previousDeltaV;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVEREULERIMPLICIT_H

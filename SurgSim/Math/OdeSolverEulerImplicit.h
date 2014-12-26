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
/// \note M(x(t), v(t)).a(t) = f(t, x(t), v(t))
/// \note This ode equation is solved as an ode of order 1 by defining the state vector y = (x v)^t:
/// \note y' = ( x' ) = ( dx/dt ) = (       v        )
/// \note      ( v' ) = ( dv/dt ) = ( M(x, v)^{-1}.f(x, v) )
/// \note y' = f(t, y)
/// \note Euler Implicit is also called backward Euler as it solves this integral using a backward evaluation:
/// \note y' = (y(t) - y(t-dt)) / dt
/// \note which leads to the integration scheme:
/// \note { x(t+dt) = x(t) + dt.v(t+dt)
/// \note { v(t+dt) = v(t) + dt.a(t+dt)
class OdeSolverEulerImplicit : public OdeSolver
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolverEulerImplicit(OdeEquation* equation);

	void solve(double dt, const OdeState& currentState, OdeState* newState) override;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVEREULERIMPLICIT_H

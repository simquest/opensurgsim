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

#ifndef SURGSIM_MATH_ODESOLVERRUNGEKUTTA4_H
#define SURGSIM_MATH_ODESOLVERRUNGEKUTTA4_H

#include <array>

#include "SurgSim/Math/OdeSolver.h"

namespace SurgSim
{

namespace Math
{

/// Runge Kutta 4 ode solver
/// See http://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
/// \note M(x(t), v(t)).a(t) = f(t, x(t), v(t))
/// \note This ode equation is solved as an ode of order 1 by defining the state vector y = (x v)^t:
/// \note y' = ( x' ) = ( dx/dt ) = (       v        )
/// \note      ( v' ) = ( dv/dt ) = ( M(x, v)^{-1}.f(x, v) )
/// \note y' = f(t, y)
/// \note Runge Kutta 4 solves it via 4 dependents evaluation of f at different times
/// \note y(n+1) = y(n) + 1/6.dt (k1 + 2*k2 + 2*k3 + k4)
/// \note with:
/// \note k1 = f(t     , y(n))
/// \note k2 = f(t+dt/2, y(n) + k1.dt/2)
/// \note k3 = f(t+dt/2, y(n) + k2.dt/2)
/// \note k4 = f(t+dt  , y(n) + k3.dt)
class OdeSolverRungeKutta4 : public OdeSolver
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolverRungeKutta4(OdeEquation* equation);

	/// Solves the equation
	/// \param dt The time step
	/// \param currentState State at time t
	/// \param[out] newState State at time t+dt
	void solve(double dt, const OdeState& currentState, OdeState* newState) override;

protected:
	/// Temporary vectors to store the 4 intermediates evaluations
	Vector m_force;

	/// Internal structure to hold the 4 temporary evaluations
	struct RungeKuttaDerivedState
	{
		RungeKuttaDerivedState(){}
		RungeKuttaDerivedState(const Vector& v, const Vector& a) : velocity(v), acceleration(a) {}
		Vector velocity;
		Vector acceleration;
	};
	/// Runge kutta 4 intermediate system evaluations
	RungeKuttaDerivedState m_k1, m_k2, m_k3, m_k4;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVERRUNGEKUTTA4_H

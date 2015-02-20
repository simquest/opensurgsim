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

/// Runge Kutta 4 ode solver (See http://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods)
/// solves the following \f$2^{nd}\f$ order ode
/// \f$M(x(t), v(t)).a(t) = f(t, x(t), v(t))\f$.
/// This ode is solved as an ode of order 1 by defining the state vector
/// \f$y = \left(\begin{array}{c}x\\v\end{array}\right)\f$:
/// \f[
///   y' = \left(\begin{array}{c} x' \\ v' \end{array}\right) =
///   \left(\begin{array}{c} v \\ M(x, v)^{-1}.f(t,x, v) \end{array}\right) =
///   f(t, y)
/// \f]
/// After integrating this equation, we get:
/// \f[ y(t+dt) - y(t) = \int_t^{t+dt} f(t,y) dt \f]
/// Runge Kutta 4 evaluates the integral term using 4 dependents evaluations of \f$f\f$ at different times and states:
/// \f[
///   \begin{array}{l}
///     y(t+dt) = y(t) + \frac{dt}{6} (k_1 + 2k_2 + 2k_3 + k_4) \\
///     \text{with:} \\
///     \quad
///     \begin{array}{lllllll}
///       k_1 &=& f(& t              &,& y(t)                    &) \\
///       k_2 &=& f(& t+\frac{dt}{2} &,& y(t) + \frac{dt}{2} k_1 &) \\
///       k_3 &=& f(& t+\frac{dt}{2} &,& y(t) + \frac{dt}{2} k_2 &) \\
///       k_4 &=& f(& t+dt           &,& y(t) + dt k_3           &)
///     \end{array}
///   \end{array}
/// \f]
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
	/// Internal structure to hold the 4 temporary evaluations
	struct RungeKuttaDerivedState
	{
		RungeKuttaDerivedState(){}
		RungeKuttaDerivedState(const Vector& v, const Vector& a) : velocity(v), acceleration(a) {}
		Vector velocity;
		Vector acceleration;
	};

	///@{
	/// Runge kutta 4 intermediate system evaluation
	RungeKuttaDerivedState m_k1, m_k2, m_k3, m_k4;
	///@}
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVERRUNGEKUTTA4_H

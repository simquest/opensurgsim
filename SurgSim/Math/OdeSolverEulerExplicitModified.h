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

#ifndef SURGSIM_MATH_ODESOLVEREULEREXPLICITMODIFIED_H
#define SURGSIM_MATH_ODESOLVEREULEREXPLICITMODIFIED_H

#include "SurgSim/Math/OdeSolver.h"

namespace SurgSim
{

namespace Math
{

/// Modified Euler Explicit ode solver solves the following \f$2^{nd}\f$ order ode
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
/// \note The modified Euler explicit is the same as Euler explicit, but simply using the newly calculated velocity
/// to update the position with instead of using the velocity from the previous time-step.
/// \note This makes this solver explicit on the velocity and implicit on the position, it improves the stability
/// of the method compare to the Euler explicit but is still an explicit method.
/// \note The numerical integration scheme becomes:
/// \f[
///   \left\{
///   \begin{array}{ccccl}
///     x(t+dt) &=& x(t) &+& dt.v(t+dt)
///     \\ v(t+dt) &=& v(t) &+& dt.a(t)
///   \end{array}
///   \right.
/// \f]
/// \sa OdeSolverEulerExplicit
class OdeSolverEulerExplicitModified : public OdeSolver
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolverEulerExplicitModified(OdeEquation* equation);

	void solve(double dt, const OdeState& currentState, OdeState* newState, bool computeCompliance = true) override;

protected:
	void assembleLinearSystem(double dt, const OdeState& state, const OdeState& newState,
		bool computeRHS = true) override;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVEREULEREXPLICITMODIFIED_H

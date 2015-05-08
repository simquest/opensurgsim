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

#ifndef SURGSIM_MATH_ODESOLVERSTATIC_H
#define SURGSIM_MATH_ODESOLVERSTATIC_H

#include "SurgSim/Math/OdeSolver.h"

namespace SurgSim
{

namespace Math
{

/// Static ode solver solves the following \f$2^{nd}\f$ order ode \f$M(x(t), v(t)).a(t) = f(t, x(t), v(t))\f$. <br>
/// This ode equation is solved w.r.t. \f$x\f$, by discarding all time derived variables (i.e. \f$v\f$, \f$a\f$)
/// reducing the equation to solve to:
/// \f[
/// 0 = f(t, x(t)) = f_{ext} + f_{int}(t, x(t)) = f_{ext} - K(x).(x - x0)
/// \f]
/// \note This solver does not solve the resulting non-linear equations, but their linearization:
/// \f$f_{ext} - K.(x - x0)=0\f$
class OdeSolverStatic : public OdeSolver
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolverStatic(OdeEquation* equation);

	void solve(double dt, const OdeState& currentState, OdeState* newState) override;

protected:
	void assembleLinearSystem(double dt, const OdeState& state, const OdeState& newState,
							  bool computeRHS = true) override;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVERSTATIC_H

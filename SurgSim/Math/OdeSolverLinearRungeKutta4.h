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

#ifndef SURGSIM_MATH_ODESOLVERLINEARRUNGEKUTTA4_H
#define SURGSIM_MATH_ODESOLVERLINEARRUNGEKUTTA4_H

#include "SurgSim/Math/OdeSolverRungeKutta4.h"

namespace SurgSim
{

namespace Math
{

/// Linear Version of the Runge Kutta 4 ode solver
/// This solver assumes that the system is linear
/// ie that Mass, Damping, and Stiffness matrices do not change.
class OdeSolverLinearRungeKutta4 : public OdeSolverRungeKutta4
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolverLinearRungeKutta4(OdeEquation* equation);

	/// The parameter computeCompliance is irrelevant for any Linear solver as it is a constant matrix.
	/// It will be precomputed on the first call and use in the following calls, no matter what the parameter value is.
	void solve(double dt, const OdeState& currentState, OdeState* newState, bool computeCompliance = true) override;

private:
	bool m_initialized;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVERLINEARRUNGEKUTTA4_H

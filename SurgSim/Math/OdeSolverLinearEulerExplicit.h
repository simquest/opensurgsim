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

#ifndef SURGSIM_MATH_ODESOLVERLINEAREULEREXPLICIT_H
#define SURGSIM_MATH_ODESOLVERLINEAREULEREXPLICIT_H

#include "SurgSim/Math/OdeSolverEulerExplicit.h"

namespace SurgSim
{

namespace Math
{

/// Linear Version of the Euler Explicit ode solver
/// This solver assumes that the system is linear, ie that Mass,
/// Damping, and Stiffness matrices do not change.
template <class State, class MT, class DT, class KT, class ST>
class LinearExplicitEuler : public ExplicitEuler<State, MT, DT, KT, ST>
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit LinearExplicitEuler(OdeEquation<State, MT, DT, KT, ST>* equation);

	/// Solves the equation
	/// \param dt The time step
	/// \param currentState State at time t
	/// \param[out] newState State at time t+dt
	virtual void solve(double dt, const State& currentState, State* newState) override;

private:
	/// Has the solver been at least once to initialize itself
	bool m_initialized;
};

}; // namespace Math

}; // namespace SurgSim


#include "SurgSim/Math/OdeSolverLinearEulerExplicit-inl.h"

#endif // SURGSIM_MATH_ODESOLVERLINEAREULEREXPLICIT_H



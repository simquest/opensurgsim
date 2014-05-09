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
/// This solver assumes that the system is linear, ie that Mass,
/// Damping, and Stiffness matrices do not change.
template <class State>
class OdeSolverLinearRungeKutta4 : public OdeSolverRungeKutta4<State>
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolverLinearRungeKutta4(OdeEquation<State>* equation);

	virtual void solve(double dt, const State& currentState, State* newState) override;

private:
	bool m_initialized;

public:
	// Variables used from OdeSolver
	using OdeSolver<State>::m_compliance;
	using OdeSolver<State>::m_equation;
	using OdeSolver<State>::m_name;

	using OdeSolverRungeKutta4<State>::m_force;
	using OdeSolverRungeKutta4<State>::m_k1;
	using OdeSolverRungeKutta4<State>::m_k2;
	using OdeSolverRungeKutta4<State>::m_k3;
	using OdeSolverRungeKutta4<State>::m_k4;
};

}; // namespace Math

}; // namespace SurgSim

#include "SurgSim/Math/OdeSolverLinearRungeKutta4-inl.h"

#endif // SURGSIM_MATH_ODESOLVERLINEARRUNGEKUTTA4_H
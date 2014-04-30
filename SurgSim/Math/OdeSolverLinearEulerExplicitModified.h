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

#ifndef SURGSIM_MATH_ODESOLVERLINEAREULEREXPLICITMODIFIED_H
#define SURGSIM_MATH_ODESOLVERLINEAREULEREXPLICITMODIFIED_H

#include "SurgSim/Math/OdeSolverEulerExplicitModified.h"

namespace SurgSim
{

namespace Math
{

/// Linear Version of the Modified Euler Explicit ode solver
/// This solver assumes that the system is linear, ie that Mass,
/// Damping, and Stiffness matrices do not change.
template <class State, class MT, class DT, class KT, class ST>
class OdeSolverLinearEulerExplicitModified : public OdeSolverEulerExplicitModified<State, MT, DT, KT, ST>
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolverLinearEulerExplicitModified(OdeEquation<State, MT, DT, KT, ST>* equation);

	virtual void solve(double dt, const State& currentState, State* newState) override;

private:
	/// Has the solver been initialized
	bool m_initialized;

public:
	// Variables used from OdeSolver
	using OdeSolver<State, MT, DT, KT, ST>::m_name;
	using OdeSolver<State, MT, DT, KT, ST>::m_equation;
	using OdeSolver<State, MT, DT, KT, ST>::m_compliance;
};

}; // namespace Math

}; // namespace SurgSim

#include "SurgSim/Math/OdeSolverLinearEulerExplicitModified-inl.h"

#endif // SURGSIM_MATH_ODESOLVERLINEAREULEREXPLICITMODIFIED_H



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

#ifndef SURGSIM_MATH_ODESOLVERLINEAREULERIMPLICIT_H
#define SURGSIM_MATH_ODESOLVERLINEAREULERIMPLICIT_H

#include "SurgSim/Math/OdeSolverEulerImplicit.h"

namespace SurgSim
{

namespace Math
{

/// Linear Version of the Euler Implicit ode solver
/// This solver assumes that the system is linear, ie that Mass,
/// Damping, and Stiffness matrices do not change.
template <class State>
class OdeSolverLinearEulerImplicit : public OdeSolverEulerImplicit<State>
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolverLinearEulerImplicit(OdeEquation<State>* equation);

	virtual void solve(double dt, const State& currentState, State* newState) override;

private:
	/// The constant stiffness matrix
	Matrix m_constantK;

	/// Has the solver been initialized
	bool m_initialized;

public:
	// Variables used from OdeSolver
	using OdeSolver<State>::m_compliance;
	using OdeSolver<State>::m_equation;
	using OdeSolver<State>::m_name;
};

}; // namespace Math

}; // namespace SurgSim

#include "SurgSim/Math/OdeSolverLinearEulerImplicit-inl.h"

#endif // SURGSIM_MATH_ODESOLVERLINEAREULERIMPLICIT_H

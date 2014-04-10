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
#include "SurgSim/Math/LinearSolveAndInverse.h"

namespace SurgSim
{

namespace Math
{

/// Static ode solver
/// \note M(x(t), v(t)).a(t) = f(t, x(t), v(t))
/// \note This ode equation is solved by discarding all time derived variables, resulting in simply solving
/// \note K.deltaX = F
/// \tparam State Type of the state y=(x v)
/// \tparam MT Type of the matrix M
/// \tparam DT Type of the matrix D
/// \tparam KT Type of the matrix K
/// \tparam ST Type of the system matrix (linear combination of M, D, K)
/// \note State is expected to hold on to the dof derivatives and have the API:
/// \note   Vector& getPositions();
/// \note   Vector& getVelocities();
/// \note   Vector& getAccelerations();
template <class State, class MT, class DT, class KT, class ST>
class Static : public OdeSolver<State, MT, DT, KT, ST>
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit Static(OdeEquation<State, MT, DT, KT, ST>* equation);

	virtual void solve(double dt, const State& currentState, State* newState) override;

private:
	/// Helper variable to solve and inverse a system of linear equations
	/// Optimized for the system matrix type
	SolveAndInverse<ST> m_solveAndInverse;

public:
	// Variables used from OdeSolver
	using OdeSolver<State, MT, DT, KT, ST>::m_name;
	using OdeSolver<State, MT, DT, KT, ST>::m_equation;
	using OdeSolver<State, MT, DT, KT, ST>::m_systemMatrix;
	using OdeSolver<State, MT, DT, KT, ST>::m_compliance;
};

}; // namespace Math

}; // namespace SurgSim

#include "SurgSim/Math/OdeSolverStatic-inl.h"

#endif // SURGSIM_MATH_ODESOLVERSTATIC_H

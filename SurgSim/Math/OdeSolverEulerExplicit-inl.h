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

#ifndef SURGSIM_MATH_ODESOLVEREULEREXPLICIT_INL_H
#define SURGSIM_MATH_ODESOLVEREULEREXPLICIT_INL_H

namespace SurgSim
{

namespace Math
{

template <class State> OdeSolverEulerExplicit<State>::OdeSolverEulerExplicit(OdeEquation<State>* equation)
	: OdeSolver<State>(equation)
{
	m_name = "Ode Solver Euler Explicit";
}

template <class State> void OdeSolverEulerExplicit<State>::solve(double dt, const State& currentState, State* newState)
{
	// General equation to solve:
	//   M.a(t) = f(t, x(t), v(t))
	// System on the velocity level:
	//   (M/dt).deltaV = f(t, x(t), v(t))

	// Computes f(t, x(t), v(t))
	const Vector& f = m_equation.computeF(currentState);

	// Computes M
	const Matrix& M = m_equation.computeM(currentState);

	// Computes the system matrix (left-hand-side matrix)
	m_systemMatrix = M * (1.0 / dt);

	// Computes deltaV (stored in the accelerations) and m_compliance = 1/m_systemMatrix
	Vector& deltaV = newState->getAccelerations();
	(*m_linearSolver)(m_systemMatrix, f, &deltaV, &m_compliance);

	// Compute the new state using the Euler Explicit scheme:
	newState->getPositions()  = currentState.getPositions()  + dt * currentState.getVelocities();
	newState->getVelocities() = currentState.getVelocities() + deltaV;

	// Adjust the acceleration variable to contain accelerations: a = deltaV/dt
	newState->getAccelerations() /= dt;
}

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVEREULEREXPLICIT_INL_H

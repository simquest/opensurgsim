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

#ifndef SURGSIM_MATH_ODESOLVEREULERIMPLICIT_INL_H
#define SURGSIM_MATH_ODESOLVEREULERIMPLICIT_INL_H

namespace SurgSim
{

namespace Math
{

template <class State, class MT, class DT, class KT, class ST>
OdeSolverEulerImplicit<State, MT, DT, KT, ST>::OdeSolverEulerImplicit(
	OdeEquation<State, MT, DT, KT, ST>* equation) :
	OdeSolver<State, MT, DT, KT, ST>(equation)
{
	m_name = "Ode Solver Euler Implicit";
}

template <class State, class MT, class DT, class KT, class ST>
void OdeSolverEulerImplicit<State, MT, DT, KT, ST>::solve(double dt, const State& currentState, State* newState)
{
	// General equation to solve:
	//   M.a(t+dt) = f(t+dt, x(t+dt), v(t+dt))
	//   M.a(t+dt) = f(t) + df/dx.deltaX + df/dv.deltaV
	// Note that K = -df/dx and D = -df/dv
	// Compliance matrix on the velocity level:
	//   (M.deltaV)/dt = f(t) - K.(dt.v(t) + dt.deltaV) - D.deltaV
	//   (M/dt + D + dt.K).deltaV = f(t) - dt.K.v(t)

	// Computes f(t, x(t), v(t)), M, D, K all at the same time
	MT* M;
	DT* D;
	KT* K;
	Vector* f;
	m_equation.computeFMDK(currentState, &f, &M, &D, &K);

	// Adds the Euler Implicit terms on the right-hand-side
	*f -= ((*K) * currentState.getVelocities()) * dt;

	// Computes the system matrix (left-hand-side matrix)
	m_systemMatrix  = (*M) * (1.0 / dt);
	m_systemMatrix += (*D);
	m_systemMatrix += (*K) * dt;

	// Computes deltaV (stored in the accelerations) and m_compliance = 1/m_systemMatrix
	Vector& deltaV = newState->getAccelerations();
	m_solveAndInverse(m_systemMatrix, *f, &deltaV, &(m_compliance));

	// Compute the new state using the Euler Implicit scheme:
	newState->getVelocities() = currentState.getVelocities() + deltaV;
	newState->getPositions()  = currentState.getPositions()  + dt * newState->getVelocities();

	// Adjust the acceleration variable to contain accelerations: a = deltaV/dt
	newState->getAccelerations() /= dt;
}

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVEREULERIMPLICIT_INL_H

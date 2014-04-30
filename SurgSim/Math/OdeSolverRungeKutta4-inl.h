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

#ifndef SURGSIM_MATH_ODESOLVERRUNGEKUTTA4_INL_H
#define SURGSIM_MATH_ODESOLVERRUNGEKUTTA4_INL_H

namespace SurgSim
{

namespace Math
{

template <class State, class MT, class DT, class KT, class ST>
OdeSolverRungeKutta4<State, MT, DT, KT, ST>::OdeSolverRungeKutta4(
	OdeEquation<State, MT, DT, KT, ST>* equation) :
	OdeSolver<State, MT, DT, KT, ST>(equation)
{
	m_name = "Ode Solver Runge Kutta 4";
}

template <class State, class MT, class DT, class KT, class ST>
void OdeSolverRungeKutta4<State, MT, DT, KT, ST>::solve(double dt, const State& currentState, State* newState)
{
	// General equation to solve:
	//   M.a(t) = F(t, x(t), v(t)), which is an ode of order 2 that can be reduced to an ode of order 1:
	// y' = (x)' = (          v          ) = f(t, y)
	//      (v)  = (M^-1.F(t, x(t), v(t)))
	// In terms of (x), f(t, (x)) = (v                    )
	//             (v)       (v)    (M^-1.F(t, x(t), v(t)))

	// Runge Kutta 4 computes y(n+1) = y(n) + 1/6.dt.(k1 + 2 * k2 + 2 * k3 + k4)
	// with k1 = f(t(n)       , y(n)            )
	// with k2 = f(t(n) + dt/2, y(n) + k1 * dt/2)
	// with k3 = f(t(n) + dt/2, y(n) + k2 * dt/2)
	// with k4 = f(t(n) + dt  , y(n) + k3 * dt  )

	// Computes M
	const MT& M = m_equation.computeM(currentState);

	// 1st evaluate k1 (note that y(n) is currentState)
	m_k1.position = currentState.getVelocities();
	m_solveAndInverse(M, m_equation.computeF(currentState), &m_k1.velocity, &(m_compliance));

	// 2nd evaluate k2
	newState->getPositions()  = currentState.getPositions()  + m_k1.position * dt / 2.0;
	newState->getVelocities() = currentState.getVelocities() + m_k1.velocity * dt / 2.0;
	m_k2.position = newState->getVelocities();
	m_solveAndInverse(M, m_equation.computeF(*newState), &m_k2.velocity, &(m_compliance));

	// 3rd evaluate k3
	newState->getPositions()  = currentState.getPositions()  + m_k2.position * dt / 2.0;
	newState->getVelocities() = currentState.getVelocities() + m_k2.velocity * dt / 2.0;
	m_k3.position = newState->getVelocities();
	m_solveAndInverse(M, m_equation.computeF(*newState), &m_k3.velocity, &(m_compliance));

	// 4th evaluate k4
	newState->getPositions()  = currentState.getPositions()  + m_k3.position * dt;
	newState->getVelocities() = currentState.getVelocities() + m_k3.velocity * dt;
	m_k4.position = newState->getVelocities();
	m_solveAndInverse(M, m_equation.computeF(*newState), &m_k4.velocity, &(m_compliance));

	// Compute the new state using Runge Kutta 4 integration scheme:
	newState->getPositions()  = currentState.getPositions();
	newState->getPositions() += (m_k1.position + m_k4.position + 2.0 * (m_k2.position + m_k3.position)) * dt / 6.0;
	newState->getVelocities() = currentState.getVelocities();
	newState->getVelocities() += (m_k1.velocity + m_k4.velocity + 2.0 * (m_k2.velocity + m_k3.velocity)) * dt / 6.0;

	// Computes the system matrix and compliance matrix
	m_systemMatrix = M / dt;
	m_compliance *= dt;
}

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVERRUNGEKUTTA4_INL_H

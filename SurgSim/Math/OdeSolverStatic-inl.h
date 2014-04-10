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

#ifndef SURGSIM_MATH_ODESOLVERSTATIC_INL_H
#define SURGSIM_MATH_ODESOLVERSTATIC_INL_H

namespace SurgSim
{

namespace Math
{

template <class State, class MT, class DT, class KT, class ST>
Static<State, MT, DT, KT, ST>::Static(
	OdeEquation<State, MT, DT, KT, ST>* equation) :
	OdeSolver<State, MT, DT, KT, ST>(equation)
{
	m_name = "Static";
}

template <class State, class MT, class DT, class KT, class ST>
void Static<State, MT, DT, KT, ST>::solve(double dt, const State& currentState, State* newState)
{
	// General equation to solve:
	//   K.deltaX = Fext + Fint(t)
	// which in the case of a linear model will derive in the expected equation:
	//   K.(x(t+dt) - x(t)) = Fext - K.(x(t) - x(0))
	//   K.(x(t+dt) - x(0)) = Fext

	// Computes f(t, x(t), v(t)) and K
	const KT& K = m_equation.computeK(currentState);
	const Vector& f = m_equation.computeF(currentState);

	m_systemMatrix = K;

	Vector& deltaX = newState->getVelocities();
	m_solveAndInverse(m_systemMatrix, f, &deltaX, &(m_compliance));

	// Compute the new state using the Euler Implicit scheme:
	newState->getPositions()  = currentState.getPositions()  + deltaX;
	// Approximate velocities as (x(t+dt) - x(t))/dt
	newState->getVelocities() = deltaX / dt;
	// Approximate accelerations as (v(t+dt) - v(t))/dt
	newState->getAccelerations() = (newState->getVelocities() - currentState.getVelocities())/ dt;
}

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVERSTATIC_INL_H

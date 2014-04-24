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

#ifndef SURGSIM_MATH_ODESOLVERLINEAREULERIMPLICIT_INL_H
#define SURGSIM_MATH_ODESOLVERLINEAREULERIMPLICIT_INL_H

namespace SurgSim
{

namespace Math
{

template <class State, class MT, class DT, class KT, class ST>
OdeSolverLinearEulerImplicit<State, MT, DT, KT, ST>::OdeSolverLinearEulerImplicit(
	OdeEquation<State, MT, DT, KT, ST>* equation) :
	OdeSolverEulerImplicit<State, MT, DT, KT, ST>(equation),
	m_initialized(false)
{
	m_name = "Ode Solver Linear Euler Implicit";
}

template <class State, class MT, class DT, class KT, class ST>
void OdeSolverLinearEulerImplicit<State, MT, DT, KT, ST>::solve(double dt, const State& currentState, State* newState)
{
	if (!m_initialized)
	{
		OdeSolverEulerImplicit<State, MT, DT, KT, ST>::solve(dt, currentState, newState);
		m_constantK = m_equation.computeK(currentState);
		m_initialized = true;
	}
	else
	{
		Vector& f = m_equation.computeF(currentState);
		f -= m_constantK * currentState.getVelocities() * dt;
		Vector deltaV = m_compliance * f;

		newState->getVelocities() = currentState.getVelocities() + deltaV;
		newState->getPositions()  = currentState.getPositions()  + dt * newState->getVelocities();
		newState->getAccelerations() = deltaV / dt;
	}
}

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVERLINEAREULERIMPLICIT_INL_H

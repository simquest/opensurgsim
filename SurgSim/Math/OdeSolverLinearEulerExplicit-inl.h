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

#ifndef SURGSIM_MATH_ODESOLVERLINEAREULEREXPLICIT_INL_H
#define SURGSIM_MATH_ODESOLVERLINEAREULEREXPLICIT_INL_H

namespace SurgSim
{

namespace Math
{

template <class State>
OdeSolverLinearEulerExplicit<State>::OdeSolverLinearEulerExplicit(OdeEquation<State>* equation)
	: OdeSolverEulerExplicit<State>(equation),
	m_initialized(false)
{
	m_name = "Ode Solver Linear Euler Explicit";
}

template <class State>
void OdeSolverLinearEulerExplicit<State>::solve(double dt, const State& currentState, State* newState)
{
	if (!m_initialized)
	{
		OdeSolverEulerExplicit<State>::solve(dt, currentState, newState);
		m_initialized = true;
	}
	else
	{
		const Vector& f = m_equation.computeF(currentState);
		Vector deltaV = m_compliance * f;

		newState->getPositions()  = currentState.getPositions()  + dt * currentState.getVelocities();
		newState->getVelocities() = currentState.getVelocities() + deltaV;
		newState->getAccelerations() = deltaV / dt;
	}
}

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVERLINEAREULEREXPLICIT_INL_H



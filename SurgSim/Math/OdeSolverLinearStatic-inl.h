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

#ifndef SURGSIM_MATH_ODESOLVERLINEARSTATIC_INL_H
#define SURGSIM_MATH_ODESOLVERLINEARSTATIC_INL_H

namespace SurgSim
{

namespace Math
{

template <class State, class MT, class DT, class KT, class ST>
LinearStatic<State, MT, DT, KT, ST>::LinearStatic(
	OdeEquation<State, MT, DT, KT, ST>* equation) :
	Static<State, MT, DT, KT, ST>(equation),
	m_initialized(false)
{
	m_name = "Linear static";
}

template <class State, class MT, class DT, class KT, class ST>
void LinearStatic<State, MT, DT, KT, ST>::solve(double dt, const State& currentState, State* newState)
{
	if (!m_initialized)
	{
		Static<State, MT, DT, KT, ST>::solve(dt, currentState, newState);
		m_initialized = true;
	}
	else
	{
		Vector& f = m_equation.computeF(currentState);
		Vector deltaX = m_compliance * f;

		// Compute the new state using the static scheme:
		newState->getPositions() = currentState.getPositions()  + deltaX;
		// Velocities are null in static mode (no time dependency)
		newState->getVelocities().setZero();
		// Accelerations are null in static mode (no time dependency)
		newState->getAccelerations().setZero();
	}
}

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVERLINEARSTATIC_INL_H

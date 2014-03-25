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

template <class State, class MT, class DT, class KT, class ST>
LinearExplicitEuler<State, MT, DT, KT, ST>::LinearExplicitEuler(
	OdeEquation<State, MT, DT, KT, ST>* equation) :
	ExplicitEuler<State, MT, DT, KT, ST>(equation)
{
	this->m_name = "Linear Explicit Euler";
	this->m_initialized = false;
}

template <class State, class MT, class DT, class KT, class ST>
void LinearExplicitEuler<State, MT, DT, KT, ST>::solve(double dt, const State& currentState, State* newState)
{
	if (! m_initialized)
	{
		ExplicitEuler<State, MT, DT, KT, ST>::solve(dt, currentState, newState);
		m_initialized = true;
	}
	else
	{
		const Vector& f = this->m_equation.computeF(currentState);
		Vector deltaV = this->m_compliance * (f);

		newState->getPositions()  = currentState.getPositions()  + dt * currentState.getVelocities();
		newState->getVelocities() = currentState.getVelocities() + deltaV;
		newState->getAccelerations() /= dt;
	}
}

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVERLINEAREULEREXPLICIT_INL_H



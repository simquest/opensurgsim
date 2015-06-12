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

#include "SurgSim/Math/OdeSolverLinearEulerExplicit.h"
#include "SurgSim/Math/OdeState.h"

namespace SurgSim
{

namespace Math
{

OdeSolverLinearEulerExplicit::OdeSolverLinearEulerExplicit(OdeEquation* equation)
	: OdeSolverEulerExplicit(equation), m_initialized(false)
{
	m_name = "Ode Solver Linear Euler Explicit";
}

void OdeSolverLinearEulerExplicit::solve(double dt, const OdeState& currentState, OdeState* newState,
		bool computeCompliance)
{
	if (!m_initialized)
	{
		// The compliance matrix is constant and used in all following calls, so we force its calculation on 1st pass.
		OdeSolverEulerExplicit::solve(dt, currentState, newState, true);
		m_initialized = true;
	}
	else
	{
		m_equation.update(currentState, ODEEQUATIONUPDATE_F);

		const Vector& f = m_equation.getF();
		Vector deltaV = m_equation.applyCompliance(currentState, f);

		newState->getPositions()  = currentState.getPositions()  + dt * currentState.getVelocities();
		newState->getVelocities() = currentState.getVelocities() + deltaV;
	}
}

}; // namespace Math

}; // namespace SurgSim

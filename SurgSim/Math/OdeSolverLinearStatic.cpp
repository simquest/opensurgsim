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

#include "SurgSim/Math/OdeSolverLinearStatic.h"
#include "SurgSim/Math/OdeState.h"

namespace SurgSim
{

namespace Math
{

OdeSolverLinearStatic::OdeSolverLinearStatic(OdeEquation* equation)
	: OdeSolverStatic(equation), m_initialized(false)
{
	m_name = "Ode Solver Linear Static";
}

void OdeSolverLinearStatic::solve(double dt, const OdeState& currentState, OdeState* newState, bool computeCompliance)
{
	if (!m_initialized)
	{
		OdeSolverStatic::solve(dt, currentState, newState, computeCompliance);
		m_initialized = true;
	}
	else
	{
		m_equation.update(currentState, true, false, false, false);

		Vector& f = m_equation.computeF(currentState);
		Vector deltaX = m_equation.applyCompliance(currentState, f);

		// Compute the new state using the static scheme:
		newState->getPositions() = currentState.getPositions()  + deltaX;
		// Velocities are null in static mode (no time dependency)
		newState->getVelocities().setZero();
	}
}

}; // namespace Math

}; // namespace SurgSim

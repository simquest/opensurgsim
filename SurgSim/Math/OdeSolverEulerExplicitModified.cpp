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

#include "SurgSim/Math/OdeSolverEulerExplicitModified.h"
#include "SurgSim/Math/OdeState.h"

namespace SurgSim
{

namespace Math
{

OdeSolverEulerExplicitModified::OdeSolverEulerExplicitModified(OdeEquation* equation)
	: OdeSolver(equation)
{
	m_name = "Ode Solver Euler Explicit Modified";
}

void OdeSolverEulerExplicitModified::solve(double dt, const OdeState& currentState, OdeState* newState)
{
	// General equation to solve:
	//   M.a(t) = f(t, x(t), v(t))
	// System on the velocity level:
	//   (M/dt).deltaV = f(t, x(t), v(t))

	// Computes f(t, x(t), v(t)) and M
	Vector& f = m_equation.computeF(currentState);
	const Matrix& M = m_equation.computeM(currentState);

	// Computes the system matrix (left-hand-side matrix)
	m_systemMatrix = M / dt;

	// Apply boundary conditions to the linear system
	currentState.applyBoundaryConditionsToVector(&f);
	currentState.applyBoundaryConditionsToMatrix(&m_systemMatrix);

	// Computes deltaV (stored in the velocities) and m_complianceMatrix = 1/m_systemMatrix
	Vector& deltaV = newState->getVelocities();
	(*m_linearSolver)(m_systemMatrix, f, &deltaV, &m_complianceMatrix);

	// Remove the boundary conditions compliance from the compliance matrix
	// This helps to prevent potential exterior LCP type calculation to violates the boundary conditions
	currentState.applyBoundaryConditionsToMatrix(&m_complianceMatrix, false);

	// Compute the new state using the Modified Euler Explicit scheme:
	newState->getVelocities() = currentState.getVelocities() + deltaV;
	newState->getPositions()  = currentState.getPositions()  + dt * newState->getVelocities();
}

}; // namespace Math

}; // namespace SurgSim

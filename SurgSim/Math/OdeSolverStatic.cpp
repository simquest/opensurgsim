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

#include "SurgSim/Math/OdeSolverStatic.h"
#include "SurgSim/Math/OdeState.h"

namespace SurgSim
{

namespace Math
{

OdeSolverStatic::OdeSolverStatic(OdeEquation* equation)
	: OdeSolver(equation)
{
	m_name = "Ode Solver Static";
}

void OdeSolverStatic::solve(double dt, const OdeState& currentState, OdeState* newState, bool computeCompliance)
{
	// General equation to solve:
	//   K.deltaX = Fext + Fint(t)
	// which in the case of a linear model will derive in the expected equation:
	//   K.(x(t+dt) - x(t)) = Fext - K.(x(t) - x(0))
	//   K.(x(t+dt) - x(0)) = Fext
	// Therefore the system matrix is K

	// Computes f(t, x(t), v(t)) and K
	const Matrix& K = m_equation.computeK(currentState);
	Vector& f = m_equation.computeF(currentState);

	m_systemMatrix = K;

	// Apply boundary conditions to the linear system
	currentState.applyBoundaryConditionsToVector(&f);
	currentState.applyBoundaryConditionsToMatrix(&m_systemMatrix);

	// Computes deltaX (stored in the positions) and m_complianceMatrix = 1/m_systemMatrix if requested
	Vector& deltaX = newState->getPositions();
	if (computeCompliance)
	{
		(*m_linearSolver)(m_systemMatrix, f, &deltaX, &m_complianceMatrix);
		// Remove the boundary conditions compliance from the compliance matrix
		// This helps to prevent potential exterior LCP type calculation to violates the boundary conditions
		currentState.applyBoundaryConditionsToMatrix(&m_complianceMatrix, false);
	}
	else
	{
		(*m_linearSolver)(m_systemMatrix, f, &deltaX);
	}

	// Compute the new state using the static scheme:
	newState->getPositions() = currentState.getPositions() + deltaX;
	// Velocities are null in static mode (no time dependency)
	newState->getVelocities().setZero();
}

void OdeSolverStatic::computeMatrices(double dt, const OdeState& state)
{
	// Computes the system matrix K
	m_systemMatrix = m_equation.computeK(state);
	state.applyBoundaryConditionsToMatrix(&m_systemMatrix);

	(*m_linearSolver)(m_systemMatrix, Vector(), nullptr, &m_complianceMatrix);
	// Remove the boundary conditions compliance from the compliance matrix
	// This helps to prevent potential exterior LCP type calculation to violates the boundary conditions
	state.applyBoundaryConditionsToMatrix(&m_complianceMatrix, false);
}

}; // namespace Math

}; // namespace SurgSim

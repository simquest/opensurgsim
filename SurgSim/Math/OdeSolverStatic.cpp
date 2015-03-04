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

void OdeSolverStatic::solve(double dt, const OdeState& currentState, OdeState* newState)
{
	// General equation to solve:
	//   K.deltaX = Fext + Fint(t)
	// which in the case of a linear model will derive in the expected equation:
	//   K.(x(t+dt) - x(t)) = Fext - K.(x(t) - x(0))
	//   K.(x(t+dt) - x(0)) = Fext

	// Computes f(t, x(t), v(t)) and K
	const SparseMatrix& K = m_equation.computeK(currentState);
	Vector& f = m_equation.computeF(currentState);

	//m_systemMatrix.resize(K.rows(), K.cols());
	m_systemMatrix = K;

	// Apply boundary conditions to the linear system
	currentState.applyBoundaryConditionsToVector(&f);
	currentState.applyBoundaryConditionsToMatrix(&m_systemMatrix);

	// Computes deltaX (stored in the positions) and m_compliance = 1/m_systemMatrix
	Vector& deltaX = newState->getPositions();
	(*m_linearSolver)(m_systemMatrix, f, &deltaX, &m_compliance);

	// Remove the boundary conditions compliance from the compliance matrix
	// This helps to prevent potential exterior LCP type calculation to violates the boundary conditions
	currentState.applyBoundaryConditionsToMatrix(&m_compliance, false);

	// Compute the new state using the static scheme:
	newState->getPositions()  = currentState.getPositions()  + deltaX;
	// Velocities are null in static mode (no time dependency)
	newState->getVelocities().setZero();
}

}; // namespace Math

}; // namespace SurgSim

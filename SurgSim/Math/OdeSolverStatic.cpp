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
	//   K.deltaX = f(t) = Fext + Fint(t)
	// which in the case of a linear model will derive in the expected equation:
	//   K.(x(t+dt) - x(t)) = Fext - K.(x(t) - x(0))
	//   K.(x(t+dt) - x(0)) = Fext
	//   systemMatrix . solution   = rhs
	// Therefore systemMatrix = K, solution = deltaX, rhs = f(t)

	// Assemble the linear system systemMatrix.solution = rhs
	assembleLinearSystem(dt, currentState, *newState);

	// Solve the linear system to find solution = deltaX
	m_solution = m_linearSolver->solve(m_rhs);

	// Compute the new state using the static scheme:
	newState->getPositions() = currentState.getPositions() + m_solution;
	// Velocities are null in static mode (no time dependency)
	newState->getVelocities().setZero();

	if (computeCompliance)
	{
		computeComplianceMatrixFromSystemMatrix(currentState);
	}
}

void OdeSolverStatic::assembleLinearSystem(double dt, const OdeState& state, const OdeState& newState, bool computeRHS)
{
	// General equation to solve:
	//   K.deltaX = Fext + Fint(t)
	// which in the case of a linear model will derive in the expected equation:
	//   K.(x(t+dt) - x(t)) = Fext - K.(x(t) - x(0))
	//   K.(x(t+dt) - x(0)) = Fext
	//   systemMatrix . solution   = rhs
	// Therefore systemMatrix = K, solution = deltaX, rhs = f(t)

	// Computes the LHS systemMatrix
	m_systemMatrix = m_equation.computeK(state);
	state.applyBoundaryConditionsToMatrix(&m_systemMatrix);

	// Feed the systemMatrix to the linear solver, so it can be used after this call to solve or inverse the matrix
	m_linearSolver->setMatrix(m_systemMatrix);

	// Computes the RHS vector
	if (computeRHS)
	{
		m_rhs = m_equation.computeF(state);
		state.applyBoundaryConditionsToVector(&m_rhs);
	}
}

}; // namespace Math

}; // namespace SurgSim

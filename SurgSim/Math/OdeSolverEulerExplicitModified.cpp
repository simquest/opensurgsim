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

void OdeSolverEulerExplicitModified::solve(double dt, const OdeState& currentState, OdeState* newState,
										   bool computeCompliance)
{
	// General equation to solve:
	//   M.a(t) = f(t, x(t), v(t))
	// Using Euler explicit { v(t+dt) = v(t) + dt.a(t)
	//                      { x(t+dt) = x(t) + dt.v(t+dt)
	// The resulting linear system on the velocity level is:
	//   (M/dt)       . deltaV   = f(t, x(t), v(t))
	//   systemMatrix . solution = rhs
	// Therefore, systemMatrix = M/dt, solution = deltaV and rhs = f

	// Assemble the linear system systemMatrix.solution = rhs
	assembleLinearSystem(dt, currentState, *newState);

	// Solve the linear system to find solution = deltaV
	m_solution = m_linearSolver->solve(m_rhs);

	// Compute the new state using the Modified Euler Explicit scheme:
	newState->getVelocities() = currentState.getVelocities() + m_solution;
	newState->getPositions()  = currentState.getPositions()  + dt * newState->getVelocities();

	if (computeCompliance)
	{
		computeComplianceMatrixFromSystemMatrix(currentState);
	}
}

void OdeSolverEulerExplicitModified::assembleLinearSystem(double dt, const OdeState& state, const OdeState& newState,
														  bool computeRHS)
{
	// General equation to solve:
	//   M.a(t) = f(t, x(t), v(t))
	// Using Euler explicit { v(t+dt) = v(t) + dt.a(t)
	//                      { x(t+dt) = x(t) + dt.v(t)
	// The resulting linear system on the velocity level is:
	//   (M/dt)       . deltaV = f(t, x(t), v(t))
	//   systemMatrix . x      = b
	// Therefore, systemMatrix = M/dt, x = deltaV and b = f

	m_equation.updateFMDK(state, ODEEQUATIONUPDATE_F | ODEEQUATIONUPDATE_M);

	// Computes the LHS systemMatrix
	m_systemMatrix = m_equation.getM() / dt;
	state.applyBoundaryConditionsToMatrix(&m_systemMatrix);

	// Feed the systemMatrix to the linear solver, so it can be used after this call to solve or inverse the matrix
	m_linearSolver->setMatrix(m_systemMatrix);

	// Computes the RHS vector
	if (computeRHS)
	{
		m_rhs = m_equation.getF();
		state.applyBoundaryConditionsToVector(&m_rhs);
	}
}

}; // namespace Math

}; // namespace SurgSim

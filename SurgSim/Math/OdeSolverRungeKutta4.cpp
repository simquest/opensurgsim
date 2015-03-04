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

#include "SurgSim/Math/OdeSolverRungeKutta4.h"
#include "SurgSim/Math/OdeState.h"

namespace SurgSim
{

namespace Math
{

OdeSolverRungeKutta4::OdeSolverRungeKutta4(OdeEquation* equation)
	: OdeSolver(equation)
{
	m_name = "Ode Solver Runge Kutta 4";
}

void OdeSolverRungeKutta4::solve(double dt, const OdeState& currentState, OdeState* newState)
{
	// General equation to solve:
	//   M.a(t) = F(t, x(t), v(t)), which is an ode of order 2 that can be reduced to an ode of order 1:
	// y' = (x)' = (          v          ) = f(t, y)
	//      (v)  = (M^-1.F(t, x(t), v(t)))
	// In terms of (x), f(t, (x)) = (v                    )
	//             (v)       (v)    (M^-1.F(t, x(t), v(t)))

	// Runge Kutta 4 computes y(n+1) = y(n) + 1/6.dt.(k1 + 2 * k2 + 2 * k3 + k4)
	// with k1 = f(t(n)       , y(n)            )
	// with k2 = f(t(n) + dt/2, y(n) + k1 * dt/2)
	// with k3 = f(t(n) + dt/2, y(n) + k2 * dt/2)
	// with k4 = f(t(n) + dt  , y(n) + k3 * dt  )

	// Computes M (stores it in m_systemMatrix to avoid dynamic re-allocation)
	SparseMatrix& M = (m_systemMatrix = m_equation.computeM(currentState));

	// Apply the boundary conditions to the mass matrix
	currentState.applyBoundaryConditionsToMatrix(&M);

	// 1st evaluate k1 (note that y(n) is currentState)
	m_k1.velocity = currentState.getVelocities();
	(*m_linearSolver)(M, *currentState.applyBoundaryConditionsToVector(&m_equation.computeF(currentState)),
					  &m_k1.acceleration, &(m_compliance));

	// Remove the boundary conditions compliance from the compliance matrix
	// This helps to prevent potential exterior LCP type calculation to violates the boundary conditions
	currentState.applyBoundaryConditionsToMatrix(&m_compliance, false);
	// Note: no need to apply the boundary conditions to any computed forces as of now because m_compliance
	// has entire lines of zero for all fixed dof. So m_compliance * F will produce the proper displacement
	// without violating any boundary conditions.

	// 2nd evaluate k2
	newState->getPositions()  = currentState.getPositions()  + m_k1.velocity * dt / 2.0;
	newState->getVelocities() = currentState.getVelocities() + m_k1.acceleration * dt / 2.0;
	m_k2.velocity = newState->getVelocities();
	m_k2.acceleration = m_compliance * m_equation.computeF(*newState);

	// 3rd evaluate k3
	newState->getPositions()  = currentState.getPositions()  + m_k2.velocity * dt / 2.0;
	newState->getVelocities() = currentState.getVelocities() + m_k2.acceleration * dt / 2.0;
	m_k3.velocity = newState->getVelocities();
	m_k3.acceleration = m_compliance * m_equation.computeF(*newState);

	// 4th evaluate k4
	newState->getPositions()  = currentState.getPositions()  + m_k3.velocity * dt;
	newState->getVelocities() = currentState.getVelocities() + m_k3.acceleration * dt;
	m_k4.velocity = newState->getVelocities();
	m_k4.acceleration = m_compliance * m_equation.computeF(*newState);

	// Compute the new state using Runge Kutta 4 integration scheme:
	newState->getPositions()  = currentState.getPositions() +
								(m_k1.velocity + m_k4.velocity + 2.0 * (m_k2.velocity + m_k3.velocity)) * dt / 6.0;
	newState->getVelocities() = currentState.getVelocities() +
								(m_k1.acceleration + m_k4.acceleration + 2.0 * (m_k2.acceleration + m_k3.acceleration)) * dt / 6.0;

	// Computes the system matrix and compliance matrix
	m_systemMatrix = M / dt;
	m_compliance *= dt;
}

}; // namespace Math

}; // namespace SurgSim

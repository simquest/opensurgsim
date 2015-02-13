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

#include "SurgSim/Math/OdeSolverEulerImplicit.h"
#include "SurgSim/Math/OdeState.h"

namespace SurgSim
{

namespace Math
{

OdeSolverEulerImplicit::OdeSolverEulerImplicit(OdeEquation* equation)
	: OdeSolver(equation), m_maximumIteration(1), m_epsilonConvergence(1e-5)
{
	m_name = "Ode Solver Euler Implicit";
}

void OdeSolverEulerImplicit::setNewtonRaphsonMaximumIteration(size_t maximumIteration)
{
	m_maximumIteration = maximumIteration;
}

size_t OdeSolverEulerImplicit::getNewtonRaphsonMaximumIteration() const
{
	return m_maximumIteration;
}

void OdeSolverEulerImplicit::setNewtonRaphsonEpsilonConvergence(double epsilonConvergence)
{
	m_epsilonConvergence = epsilonConvergence;
}

double OdeSolverEulerImplicit::getNewtonRaphsonEpsilonConvergence() const
{
	return m_epsilonConvergence;
}

void OdeSolverEulerImplicit::solve(double dt, const OdeState& currentState, OdeState* newState)
{
	// General equation to solve:
	//   M.a(t+dt) = f(t+dt, x(t+dt), v(t+dt))
	//   M.a(t+dt) = f(t) + df/dx.deltaX + df/dv.deltaV
	// Note that K = -df/dx and D = -df/dv
	// Compliance matrix on the velocity level:
	//   (M.deltaV)/dt = f(t) - K.(dt.v(t) + dt.deltaV) - D.deltaV
	//   (M/dt + D + dt.K).deltaV = f(t) - dt.K.v(t)

	// Note that this system is non-linear as K and D are  non-linear in absence of information on the nature
	// of the model. We use a Newton-Raphson algorithm (http://en.wikipedia.org/wiki/Newton%27s_method) to solve
	// this non-linear problem. Note that each iteration will re-evaluate the complete system (forces and matrices).
	// Also note that this method converges quadratically around the root. In our case, the solution is deltaV, which
	// should be close to 0. This makes our problem well suited for this method.

	m_deltaV.resize(currentState.getPositions().size());
	if (m_maximumIteration > 1)
	{
		m_previousDeltaV = Vector::Zero(m_deltaV.size());
	}

	// Prepare the newState to be used in the loop, it starts as the current state.
	*newState = currentState;

	size_t numIteration = 0;
	while (numIteration < m_maximumIteration)
	{
		Matrix* M;
		Matrix* D;
		Matrix* K;
		Vector* f;

		// Computes f(t, x(t), v(t)), M, D, K all at the same time
		m_equation.computeFMDK(*newState, &f, &M, &D, &K);

		// Adds the Euler Implicit terms on the right-hand-side
		*f -= ((*K) * newState->getVelocities()) * dt;

		// Computes the system matrix (left-hand-side matrix)
		m_systemMatrix  = (*M) * (1.0 / dt);
		m_systemMatrix += (*D);
		m_systemMatrix += (*K) * dt;

		// Apply boundary conditions to the linear system
		currentState.applyBoundaryConditionsToVector(f);
		currentState.applyBoundaryConditionsToMatrix(&m_systemMatrix);

		// Computes deltaV
		(*m_linearSolver)(m_systemMatrix, *f, &m_deltaV);

		// Compute the new state using the Euler Implicit scheme:
		newState->getVelocities() = currentState.getVelocities() + m_deltaV;
		newState->getPositions()  = currentState.getPositions() + dt * newState->getVelocities();

		if (m_maximumIteration > 1)
		{
			double solutionVariation = (m_deltaV - m_previousDeltaV).norm();

			if (solutionVariation < m_epsilonConvergence)
			{
				break;
			}

			m_previousDeltaV = m_deltaV;
		}

		numIteration++;
	}

	// Only compute the compliance matrix once, around the root
	(*m_linearSolver)(m_systemMatrix, Vector(), nullptr, &m_compliance);

	// Remove the boundary conditions compliance from the compliance matrix
	// This helps to prevent potential exterior LCP type calculation to violates the boundary conditions
	currentState.applyBoundaryConditionsToMatrix(&m_compliance, false);
}

}; // namespace Math

}; // namespace SurgSim

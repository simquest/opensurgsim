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

void OdeSolverEulerImplicit::solve(double dt, const OdeState& currentState, OdeState* newState, bool computeCompliance)
{
	// General equation to solve:
	//   M.a(t+dt) = f(t+dt, x(t+dt), v(t+dt))
	// Let's note K = -df/dx and D = -df/dv.
	//  For a single linearization, we get the following linear system:
	//  M/dt.deltaV = f(t, x(t), v(t)) - K.deltaX - D.deltaV
	//  (M/dt + D + dt.K).deltaV = f(t, x(t), v(t)) - dt.K.v(t)
	// Therefore the system matrix is (M/dt + D + dt.K)

	// Note that the resulting system is non-linear as K and D are  non-linear in absence of information on the nature
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

	// See the class doxygen documentation (.dox) for explanation of the algorithm.
	// * currentState is y(t) = (x(t), v(t)).
	// * newState is the current estimate, y_n = (x_n, v_n) (with y_0 = y(t))
	// * Each iteration search for the next estimate y_{n+1} = (x_{n+1}, v_{n+1}).
	size_t numIteration = 0;
	while (numIteration < m_maximumIteration)
	{
		Matrix* M;
		Matrix* D;
		Matrix* K;
		Vector* f;

		// Computes f(t, x(t), v(t)), M, D, K all at the same time
		m_equation.computeFMDK(*newState, &f, &M, &D, &K);

		// Adds the Euler Implicit/Newton-Raphson terms on the right-hand-side
		*f += (*K) * (newState->getPositions() - currentState.getPositions() - newState->getVelocities() * dt);
		*f -= ((*M) * (newState->getVelocities() - currentState.getVelocities())) / dt;

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
		newState->getVelocities() += m_deltaV;
		newState->getPositions()  = currentState.getPositions() + dt * newState->getVelocities();

		if (m_maximumIteration > 1)
		{
			// Use the infinity norm, to treat models with small or large number of dof the same way.
			double solutionVariation = (m_deltaV - m_previousDeltaV).lpNorm<Eigen::Infinity>();

			if (solutionVariation < m_epsilonConvergence)
			{
				break;
			}

			m_previousDeltaV = m_deltaV;
		}

		numIteration++;
	}

	// The compliance matrix (if requested) is computed w.r.t. the latest state.
	if (computeCompliance)
	{
		(*m_linearSolver)(m_systemMatrix, Vector(), nullptr, &m_complianceMatrix);
		// Remove the boundary conditions compliance from the compliance matrix
		// This helps to prevent potential exterior LCP type calculation to violates the boundary conditions
		currentState.applyBoundaryConditionsToMatrix(&m_complianceMatrix, false);
	}
}

void OdeSolverEulerImplicit::computeMatrices(double dt, const OdeState& state)
{
	// Computes the system matrix M/dt + D + dt.K
	Vector* f;
	Matrix* M, * D, * K;
	m_equation.computeFMDK(state, &f, &M, &D, &K);
	m_systemMatrix  = (*M) * (1.0 / dt);
	m_systemMatrix += (*D);
	m_systemMatrix += (*K) * dt;
	state.applyBoundaryConditionsToMatrix(&m_systemMatrix);

	// Computes the compliance matrix as the inverse of the system matrix
	(*m_linearSolver)(m_systemMatrix, Vector(), nullptr, &m_complianceMatrix);
	// Remove the boundary conditions compliance from the compliance matrix
	// This helps to prevent potential exterior LCP type calculation to violates the boundary conditions
	state.applyBoundaryConditionsToMatrix(&m_complianceMatrix, false);
}


}; // namespace Math

}; // namespace SurgSim

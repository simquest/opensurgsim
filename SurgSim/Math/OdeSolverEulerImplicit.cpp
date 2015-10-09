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
	SURGSIM_ASSERT(maximumIteration >= 1) << "The maximum iteration needs to be at least 1";

	m_maximumIteration = maximumIteration;
}

size_t OdeSolverEulerImplicit::getNewtonRaphsonMaximumIteration() const
{
	return m_maximumIteration;
}

void OdeSolverEulerImplicit::setNewtonRaphsonEpsilonConvergence(double epsilonConvergence)
{
	SURGSIM_ASSERT(epsilonConvergence >= 0.0) << "The epsilon convergence cannot be negative";

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
	// Using Euler implicit { v(t+dt) = v(t) + dt.a(t+dt)
	//                      { x(t+dt) = x(t) + dt.v(t+dt)
	// For a single linearization, we get the following linear system:
	//  M/dt.deltaV = f(t, x(t), v(t)) - K.deltaX - D.deltaV
	//  (M/dt + D + dt.K) . deltaV   = f(t, x(t), v(t)) - dt.K.v(t)
	//  systemMatrix      . solution = rhs
	// Therefore systemMatrix = (M/dt + D + dt.K), solution = deltaV, rhs = f - dt.K.v(t)
	// More terms are coming from the Newton-Raphson iterations (see class OdeSolverEulerImplicit doxygen doc for
	// more details)

	// Note that the resulting system is non-linear as K and D are  non-linear in absence of information on the nature
	// of the model. We use a Newton-Raphson algorithm (http://en.wikipedia.org/wiki/Newton%27s_method) to solve
	// this non-linear problem. Note that each iteration will re-evaluate the complete system (forces and matrices).
	// Also note that this method converges quadratically around the root. In our case, the solution is deltaV, which
	// should be close to 0. This makes our problem well suited for this method.

	if (m_maximumIteration > 1)
	{
		m_previousSolution = Vector::Zero(currentState.getNumDof());
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
		// Assemble the linear system systemMatrix*solution = rhs
		assembleLinearSystem(dt, currentState, *newState);

		// Solve the linear system to find solution = deltaV
		m_solution = m_linearSolver->solve(m_rhs);

		// Compute the new state using the Euler Implicit scheme:
		newState->getVelocities() += m_solution;
		newState->getPositions()  = currentState.getPositions() + dt * newState->getVelocities();

		if (m_maximumIteration > 1)
		{
			// Use the infinity norm, to treat models with small or large number of dof the same way.
			double solutionVariation = (m_solution - m_previousSolution).lpNorm<Eigen::Infinity>();

			if (solutionVariation < m_epsilonConvergence)
			{
				break;
			}

			m_previousSolution = m_solution;
		}

		numIteration++;
	}

	// The compliance matrix (if requested) is computed w.r.t. the latest state.
	if (computeCompliance)
	{
		computeComplianceMatrixFromSystemMatrix(currentState);
	}
}

void OdeSolverEulerImplicit::assembleLinearSystem(double dt, const OdeState& state, const OdeState& newState,
		bool computeRHS)
{
	// General equation to solve:
	//   M.a(t+dt) = f(t+dt, x(t+dt), v(t+dt))
	// Let's note K = -df/dx and D = -df/dv.
	// Using Euler implicit { v(t+dt) = v(t) + dt.a(t+dt)
	//                      { x(t+dt) = x(t) + dt.v(t+dt)
	// For a single linearization, we get the following linear system:
	//  M/dt.deltaV = f(t, x(t), v(t)) - K.deltaX - D.deltaV
	//  (M/dt + D + dt.K) . deltaV   = f(t, x(t), v(t)) - dt.K.v(t)
	//  systemMatrix      . solution = rhs
	// Therefore systemMatrix = (M/dt + D + dt.K), solution = deltaV, rhs = f - dt.K.v(t)
	// More terms are coming from the Newton-Raphson iterations (see class OdeSolverEulerImplicit doxygen doc for
	// more details)

	m_equation.updateFMDK(newState, ODEEQUATIONUPDATE_FMDK);

	const SparseMatrix& M = m_equation.getM();
	const SparseMatrix& D = m_equation.getD();
	const SparseMatrix& K = m_equation.getK();
	const Vector& f = m_equation.getF();

	// Computes the LHS systemMatrix
	m_systemMatrix  = M * (1.0 / dt);
	m_systemMatrix += D;
	m_systemMatrix += K * dt;
	state.applyBoundaryConditionsToMatrix(&m_systemMatrix);

	// Feed the systemMatrix to the linear solver, so it can be used after this call to solve or inverse the matrix
	m_linearSolver->setMatrix(m_systemMatrix);

	// Computes the RHS vector by adding the Euler Implicit/Newton-Raphson terms
	if (computeRHS)
	{
		m_rhs = f + K * (newState.getPositions() - state.getPositions() - newState.getVelocities() * dt);
		m_rhs -= (M * (newState.getVelocities() - state.getVelocities())) / dt;
		state.applyBoundaryConditionsToVector(&m_rhs);
	}
}

}; // namespace Math

}; // namespace SurgSim

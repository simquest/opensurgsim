// This file is a part of the OpenSurgSim project.
// Copyright 2013-2017, SimQuest Solutions Inc.
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

#include "Modules/Parallel/OdeSolverEulerImplicitOpenCl.h"
#include "SurgSim/Math/OdeEquation.h"

#include <viennacl/vector.hpp>
#include <viennacl/matrix.hpp>
#include <viennacl/compressed_matrix.hpp>
#include <viennacl/linalg/prod.hpp>
#include <viennacl/linalg/cg.hpp>

namespace SurgSim
{
namespace Parallel
{

OdeSolverEulerImplicitOpenCl::OdeSolverEulerImplicitOpenCl(Math::OdeEquation* odeEquation)
	: OdeSolver(odeEquation)
{

}

void OdeSolverEulerImplicitOpenCl::solve(double dt, const Math::OdeState& currentState, Math::OdeState* newState,
		bool computeCompliance /*= true*/)
{

	// Prepare the newState to be used in the loop, it starts as the current state.
	*newState = currentState;

	// Assemble the linear system systemMatrix*solution = rhs
	assembleLinearSystem(dt, currentState, *newState);


	viennacl::compressed_matrix<double> vcl_systemMatrix;
	viennacl::vector<double> vcl_rhs(m_rhs.size());
	viennacl::vector<double> vcl_solution(m_rhs.size());

	viennacl::copy(m_systemMatrix, vcl_systemMatrix);
	viennacl::copy(m_rhs, vcl_rhs);

	// Solve the linear system to find solution = deltaV
	vcl_solution = viennacl::linalg::solve(vcl_systemMatrix, vcl_rhs, viennacl::linalg::cg_tag());

	m_solution.resize(m_rhs.size());
	viennacl::copy(vcl_solution, m_solution);

	// Compute the new state using the Euler Implicit scheme:
	newState->getVelocities() += m_solution;
	newState->getPositions() = currentState.getPositions() + dt * newState->getVelocities();

	// The compliance matrix (if requested) is computed w.r.t. the latest state.
	if (computeCompliance)
	{
		computeComplianceMatrixFromSystemMatrix(currentState);
	}
}

void OdeSolverEulerImplicitOpenCl::assembleLinearSystem(double dt, const Math::OdeState& state,
		const Math::OdeState& newState, bool computeRHS /*= true*/)
{

	m_equation.updateFMDK(newState, Math::ODEEQUATIONUPDATE_FMDK);

	const Math::SparseMatrix& M = m_equation.getM();
	const Math::SparseMatrix& D = m_equation.getD();
	const Math::SparseMatrix& K = m_equation.getK();
	const Math::Vector& f = m_equation.getF();

	// Computes the LHS systemMatrix
	m_systemMatrix = M * (1.0 / dt);
	m_systemMatrix += D;
	m_systemMatrix += K * dt;
	state.applyBoundaryConditionsToMatrix(&m_systemMatrix);

	// Feed the systemMatrix to the linear solver, so it can be used after this call to solve or inverse the matrix

	// push SystemMatrix to openCl buffer ...
	m_linearSolver->setMatrix(m_systemMatrix);


	// Computes the RHS vector by adding the Euler Implicit/Newton-Raphson terms
	if (computeRHS)
	{
		m_rhs = f + K * (newState.getPositions() - state.getPositions() - newState.getVelocities() * dt);
		m_rhs -= (M * (newState.getVelocities() - state.getVelocities())) / dt;
		state.applyBoundaryConditionsToVector(&m_rhs);
	}
}

}
}

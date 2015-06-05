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

#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/OdeState.h"

namespace SurgSim
{

namespace Math
{

OdeSolver::OdeSolver(OdeEquation* equation) : m_equation(*equation)
{
	allocate(m_equation.getInitialState()->getPositions().size());

	// Default linear solver
	setLinearSolver(std::make_shared<LinearSparseSolveAndInverseLU>());
}

const std::string OdeSolver::getName() const
{
	return m_name;
}

void OdeSolver::setLinearSolver(std::shared_ptr<LinearSparseSolveAndInverse> linearSolver)
{
	m_linearSolver = linearSolver;
}

std::shared_ptr<LinearSparseSolveAndInverse> OdeSolver::getLinearSolver() const
{
	return m_linearSolver;
}

const SparseMatrix& OdeSolver::getSystemMatrix() const
{
	return m_systemMatrix;
}

const Matrix& OdeSolver::getComplianceMatrix() const
{
	return m_complianceMatrix;
}

void OdeSolver::allocate(size_t size)
{
	m_systemMatrix.resize(static_cast<SparseMatrix::Index>(size), static_cast<SparseMatrix::Index>(size));
	m_complianceMatrix.resize(size, size);
	m_solution.resize(size);
	m_rhs.resize(size);
}

void OdeSolver::computeMatrices(double dt, const OdeState& state, bool computeCompliance)
{
	/// Compute the system matrix (and discard the RHS calculation)
	assembleLinearSystem(dt, state, state, false);

	/// Compute the compliance matrix
	if (computeCompliance)
	{
		computeComplianceMatrixFromSystemMatrix(state);
	}
}

void OdeSolver::computeComplianceMatrixFromSystemMatrix(const OdeState& state)
{
	// The compliance matrix is the inverse of the system matrix
	m_complianceMatrix = m_linearSolver->getInverse();
	// The boundary conditions needs to be set on the compliance matrix and no compliance should be used for the nodes
	// Which means that the compliance matrix has entire rows and columns of zeros for the boundary conditions.
	state.applyBoundaryConditionsToMatrix(&m_complianceMatrix, false);
}

}; // namespace Math

}; // namespace SurgSim

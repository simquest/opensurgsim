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

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/LinearSparseSolveAndInverse.h"

namespace SurgSim
{

namespace Math
{

void LinearSparseSolveAndInverseLU::setMatrix(const SparseMatrix& matrix)
{
	SURGSIM_ASSERT(matrix.cols() == matrix.rows()) << "Cannot inverse a non square matrix";
	m_solver.compute(matrix);
}

Matrix LinearSparseSolveAndInverseLU::solve(const Matrix& b) const
{
	return m_solver.solve(b);
}

Matrix LinearSparseSolveAndInverseLU::getInverse() const
{
	return (m_solver.solve(Matrix::Identity(m_solver.rows(), m_solver.cols())));
}

void LinearSparseSolveAndInverseCG::setTolerance(double tolerance)
{
	m_solver.setTolerance(tolerance);
}

double LinearSparseSolveAndInverseCG::getTolerance()
{
	return m_solver.tolerance();
}

void LinearSparseSolveAndInverseCG::setMaxIterations(SparseMatrix::Index iterations)
{
	m_solver.setMaxIterations(iterations);
}

SparseMatrix::Index LinearSparseSolveAndInverseCG::getMaxIterations()
{
	return m_solver.maxIterations();
}

void LinearSparseSolveAndInverseCG::setMatrix(const SparseMatrix& matrix)
{
	SURGSIM_ASSERT(matrix.cols() == matrix.rows()) << "Cannot inverse a non square matrix";
	m_solver.compute(matrix);
}

Matrix LinearSparseSolveAndInverseCG::solve(const Matrix& b) const
{
	return m_solver.solve(b);
}

Matrix LinearSparseSolveAndInverseCG::getInverse() const
{
	return (m_solver.solve(Matrix::Identity(m_solver.rows(), m_solver.cols())));
}

}; // namespace Math

}; // namespace SurgSim

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


Matrix LinearSparseSolveAndInverse::getInverse() const
{
	return m_matrix.toDense().inverse();
}

void LinearSparseSolveAndInverseLU::setMatrix(const SparseMatrix& matrix)
{
	SURGSIM_ASSERT(matrix.cols() == matrix.rows()) << "Cannot inverse a non square matrix";
	bool sameMatrix = false;

	const auto* innerIndices = matrix.innerIndexPtr();
	const auto* outerIndices = matrix.outerIndexPtr();
	const auto* storedInnerIndices = m_matrix.innerIndexPtr();
	const auto* storedOuterIndices = m_matrix.outerIndexPtr();
	if ((matrix.outerSize() == m_matrix.outerSize()) && (matrix.innerSize() == m_matrix.innerSize()) &&
		(matrix.outerSize() > 1))
	{
		sameMatrix = true;
		for (Eigen::Index outerLoop = 0; outerLoop < matrix.outerSize() - 1; ++outerLoop)
		{
			if ((outerIndices[outerLoop] != storedOuterIndices[outerLoop]) ||
				(outerIndices[outerLoop + 1] != storedOuterIndices[outerLoop + 1]))
			{
				sameMatrix = false;
				break;
			}
			for (auto innerLoop = outerIndices[outerLoop];
				innerLoop < outerIndices[outerLoop + 1];
				++innerLoop)
			{
				if (innerIndices[innerLoop] != storedInnerIndices[innerLoop])
				{
					sameMatrix = false;
					break;
				}
			}
			if (!sameMatrix)
			{
				break;
			}
		}
	}
	
	if (sameMatrix)
	{
		m_solver.factorize(matrix);
	}
	else
	{
		m_solver.compute(matrix);
		m_matrix = matrix;
	}

	SURGSIM_ASSERT(m_solver.info() == Eigen::Success) << m_solver.lastErrorMessage();

	if (m_identity.cols() != matrix.cols() || m_identity.rows() != matrix.rows())
	{
		m_identity.resize(matrix.cols(), matrix.rows());
		m_identity.setIdentity();
	}
}

Matrix LinearSparseSolveAndInverseLU::getInverse() const
{
	// HS-5/24/2017 m_identity is Dense, if it sparse there is a reallocation when we return a dense matrix here
	return m_solver.solve(m_identity);
}

Matrix LinearSparseSolveAndInverseLU::solve(const Matrix& b) const
{
	return m_solver.solve(b);
}

void LinearSparseSolveAndInverseCG::setTolerance(double tolerance)
{
	m_solver.setTolerance(tolerance);
}

double LinearSparseSolveAndInverseCG::getTolerance()
{
	return m_solver.tolerance();
}

void LinearSparseSolveAndInverseCG::setMaxIterations(Eigen::Index iterations)
{
	m_solver.setMaxIterations(iterations);
}

Eigen::Index LinearSparseSolveAndInverseCG::getMaxIterations()
{
	return m_solver.maxIterations();
}

void LinearSparseSolveAndInverseCG::setMatrix(const SparseMatrix& matrix)
{
	SURGSIM_ASSERT(matrix.cols() == matrix.rows()) << "Cannot inverse a non square matrix";
	m_solver.compute(matrix);
	m_matrix = matrix;
}

Matrix LinearSparseSolveAndInverseCG::solve(const Matrix& b) const
{
	return m_solver.solve(b);
}

}; // namespace Math

}; // namespace SurgSim

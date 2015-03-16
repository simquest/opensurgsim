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

#include "SurgSim/Math/LinearSolveAndInverse.h"

namespace SurgSim
{

namespace Math
{

void LinearSolveAndInverseDiagonalMatrix::setMatrix(const Matrix& matrix)
{
	SURGSIM_ASSERT(matrix.cols() == matrix.rows()) << "Cannot inverse a non square matrix";

	m_inverseDiagonal = matrix.diagonal().cwiseInverse();
}

Vector LinearSolveAndInverseDiagonalMatrix::solve(const Vector& b)
{
	return m_inverseDiagonal.cwiseProduct(b);
}

Matrix LinearSolveAndInverseDiagonalMatrix::getInverse()
{
	return m_inverseDiagonal.asDiagonal();
}

void LinearSolveAndInverseDenseMatrix::setMatrix(const Matrix& matrix)
{
	SURGSIM_ASSERT(matrix.cols() == matrix.rows()) << "Cannot inverse a non square matrix";

	m_luDecomposition = matrix.partialPivLu();
}

Vector LinearSolveAndInverseDenseMatrix::solve(const Vector& b)
{
	return m_luDecomposition.solve(b);
}

Matrix LinearSolveAndInverseDenseMatrix::getInverse()
{
	return m_luDecomposition.inverse();
}

}; // namespace Math

}; // namespace SurgSim

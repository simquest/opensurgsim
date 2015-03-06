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

void LinearSolveAndInverseDiagonalMatrix::operator ()(const Matrix& A, const Vector& b, Vector* x, Matrix* Ainv)
{
	SURGSIM_ASSERT(A.cols() == A.rows()) << "Cannot inverse a non square matrix";

	if (Ainv != nullptr)
	{
		if (Ainv->cols() != A.cols() || Ainv->rows() != A.cols())
		{
			Ainv->resize(A.rows(), A.cols());
		}
		Ainv->setZero();
		Ainv->diagonal() = A.diagonal().cwiseInverse();

		if (x != nullptr)
		{
			(*x) = Ainv->diagonal().cwiseProduct(b);
		}
	}
	else if (x != nullptr)
	{
		(*x) = A.diagonal().cwiseInverse().cwiseProduct(b);
	}
}

void LinearSolveAndInverseDiagonalMatrix::update(const Matrix& A)
{
	m_inverseDiagonal = A.diagonal().cwiseInverse();
}

void LinearSolveAndInverseDiagonalMatrix::solve(const Vector& b, Vector* x)
{
	*x = m_inverseDiagonal.cwiseProduct(b);
}

void LinearSolveAndInverseDiagonalMatrix::getInverse(Matrix* Ainv)
{
	Ainv->setZero();
	Ainv->diagonal() = m_inverseDiagonal;
}

void LinearSolveAndInverseDenseMatrix::operator ()(const Matrix& A, const Vector& b, Vector* x, Matrix* Ainv)
{
	SURGSIM_ASSERT(A.cols() == A.rows()) << "Cannot inverse a non square matrix";

	if (x != nullptr || Ainv != nullptr)
	{
		const Eigen::PartialPivLU<typename Eigen::MatrixBase<Matrix>::PlainObject> lu = A.partialPivLu();
		if (x != nullptr)
		{
			(*x) = lu.solve(b);
		}
		if (Ainv != nullptr)
		{
			(*Ainv) = lu.inverse();
		}
	}
}

void LinearSolveAndInverseDenseMatrix::update(const Matrix& A)
{
	SURGSIM_ASSERT(A.cols() == A.rows()) << "Cannot inverse a non square matrix";

	m_luDecomposition = A.partialPivLu();
}

void LinearSolveAndInverseDenseMatrix::solve(const Vector& b, Vector* x)
{
	*x = m_luDecomposition.solve(b);
}

void LinearSolveAndInverseDenseMatrix::getInverse(Matrix* Ainv)
{
	*Ainv = m_luDecomposition.inverse();
}

}; // namespace Math

}; // namespace SurgSim

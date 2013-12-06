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

void SolveAndInverse<DiagonalMatrix>::operator ()
	(const DiagonalMatrix& A, const Vector& b, Vector* x, Matrix* Ainv)
{
	(*Ainv) = m_Ainv = A.inverse();
	(*x) = m_Ainv * b;
}

void SolveAndInverse<Matrix>::operator ()
	(const Matrix& A, const Vector& b, Vector* x, Matrix* Ainv)
{
	Eigen::PartialPivLU<Matrix> lu(A);
	(*x) = lu.solve(b);
	(*Ainv) = lu.inverse();
}

void SolveAndInverse<Eigen::SparseMatrix<double,Eigen::ColMajor>>::operator ()
	(const Eigen::SparseMatrix<double,Eigen::ColMajor>& A, const Vector& b, Vector* x, Matrix* Ainv)
{
	Eigen::SparseLU<const Eigen::SparseMatrix<double, Eigen::ColMajor>> solver;
	// Compute the ordering permutation vector from the structural pattern of A
	solver.analyzePattern(A);
	// Compute the numerical factorization
	solver.factorize(A);
	//Use the factors to solve the linear system
	(*x) = solver.solve(b);

	if (m_identity.rows() != A.rows())
	{
		m_identity.resize(A.rows(), A.cols());
		m_identity.setIdentity();
	}
	(*Ainv) = solver.solve(m_identity);
}

}; // namespace Math

}; // namespace SurgSim

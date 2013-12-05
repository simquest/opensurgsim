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

#ifndef SURGSIM_MATH_LINEARSOLVEANDINVERSE_H
#define SURGSIM_MATH_LINEARSOLVEANDINVERSE_H

#include "SurgSim/Framework/Assert.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace SurgSim
{

namespace Math
{

/// SolveAndInverse aims at performing an efficient linear system resolution and
/// calculating its inverse matrix at the same time.
/// The efficiency comes from specializing the linear system matrix type and use the best algorithm that fits this type
/// This class is very useful in the OdeSolver resolution to improve performance.
/// \sa SurgSim::Math::OdeSolver
/// \tparam LSM Linear System Matrix type
template <typename LSM>
class SolveAndInverse
{
public:

	/// Solve a linear system A.x=b and compute the matrix A^-1
	/// \param A Linear system matrix
	/// \param b Linear system right-hand-side
	/// \param[out] x Linear system unknown
	/// \param[out] Ainv Linear system matrix inverse = A^-1
	/// \note Ainv is of generic type Matrix as in most cases, it will be a dense matrix (except for diagonal matrix)
	void operator ()(const LSM& A, const Vector& b, Vector* x, Matrix* Ainv)
	{
		SURGSIM_ASSERT(false) << "Unknown matrix type for SolveAndInverse";
	}
};

/// Specialization for DiagonalMatrix type
template <>
class SolveAndInverse<DiagonalMatrix>
{
private:
	/// Temporary inverse matrix of type DiagonalMatrix to improve the performance of the system resolution
	DiagonalMatrix m_Ainv;

public:
	/// Solve a linear system A.x=b and compute the matrix A^-1
	/// \param A Linear system matrix
	/// \param b Linear system right-hand-side
	/// \param[out] x Linear system unknown
	/// \param[out] Ainv Linear system matrix inverse = A^-1
	/// \note Ainv is of generic type Matrix as in most cases, it will be a dense matrix (except for diagonal matrix)
	void operator ()(const DiagonalMatrix& A, const Vector& b, Vector* x, Matrix* Ainv);
};

/// Specialization for dense Matrix type
template <>
class SolveAndInverse<Matrix>
{
public:
	/// Solve a linear system A.x=b and compute the matrix A^-1
	/// \param A Linear system matrix
	/// \param b Linear system right-hand-side
	/// \param[out] x Linear system unknown
	/// \param[out] Ainv Linear system matrix inverse = A^-1
	/// \note Ainv is of generic type Matrix as in most cases, it will be a dense matrix (except for diagonal matrix)
	void operator ()(const Matrix& A, const Vector& b, Vector* x, Matrix* Ainv);
};


/// Specialization for SparseMatrix type
template <>
class SolveAndInverse<Eigen::SparseMatrix<double,Eigen::ColMajor>>
{
private:
	/// Intermediate identity matrix useful to compute the inverse by solving A.X = I
	/// \note This matrix will be allocated and initialized on 1st call, only reallocated if the system changes size
	Matrix m_identity;

public:
	/// Solve a linear system A.x=b and compute the matrix A^-1
	/// \param A Linear system matrix
	/// \param b Linear system right-hand-side
	/// \param[out] x Linear system unknown
	/// \param[out] Ainv Linear system matrix inverse = A^-1
	/// \note Ainv is of generic type Matrix as in most cases, it will be a dense matrix (except for diagonal matrix)
	void operator ()(const Eigen::SparseMatrix<double,Eigen::ColMajor>& A, const Vector& b, Vector* x, Matrix* Ainv);
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_LINEARSOLVEANDINVERSE_H

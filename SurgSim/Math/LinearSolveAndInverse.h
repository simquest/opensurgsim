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

namespace SurgSim
{

namespace Math
{

/// LinearSolveAndInverse aims at performing an efficient linear system resolution and
/// calculating its inverse matrix at the same time.
/// This class is very useful in the OdeSolver resolution to improve performance.
/// \sa SurgSim::Math::OdeSolver
class LinearSolveAndInverse
{
public:
	virtual ~LinearSolveAndInverse(){}

	/// Solve a linear system A.x=b and compute the matrix A^-1
	/// \param A Linear system matrix
	/// \param b Linear system right-hand-side
	/// \param[out] x Linear system unknown (if requested)
	/// \param[out] Ainv Linear system matrix inverse = A^-1 (if requested)
	virtual void operator ()(const Matrix& A, const Vector& b, Vector* x = nullptr, Matrix* Ainv = nullptr) = 0;
};

/// Derivation for dense matrix type
class LinearSolveAndInverseDenseMatrix : public LinearSolveAndInverse
{
public:
	void operator ()(const Matrix& A, const Vector& b, Vector* x = nullptr, Matrix* Ainv = nullptr) override;
};

/// Derivation for diagonal matrix type
class LinearSolveAndInverseDiagonalMatrix : public LinearSolveAndInverse
{
public:
	void operator ()(const Matrix& A, const Vector& b, Vector* x = nullptr, Matrix* Ainv = nullptr) override;
};

/// Derivation for tri-diagonal block matrix type
/// \tparam BlockSize Define the block size of the tri-diagonal block matrix
template <size_t BlockSize>
class LinearSolveAndInverseTriDiagonalBlockMatrix : public LinearSolveAndInverse
{
public:
	void operator ()(const Matrix& A, const Vector& b, Vector* x = nullptr, Matrix* Ainv = nullptr) override;

protected:
	/// Computes the inverse matrix
	/// \param A The matrix to inverse
	/// \param[out] inv The inverse matrix
	/// \param isSymmetric True if the matrix is symmetric, False otherwise
	/// \note isSymmetric is only indicative and helps optimizing the computation when the matrix is symmetric.
	/// \note On the other side, if the flag is true and the matrix is not symmetric, the result will be wrong.
	/// \note Assert on inverse matrix pointer (inv), on the matrix being square and on
	/// \note proper size matching between the matrix size and the blockSize
	void inverseTriDiagonalBlock(const SurgSim::Math::Matrix& A, SurgSim::Math::Matrix* inv, bool isSymmetric = false);

	/// Member variable to hold the inverse matrix in case only the solving is requested
	Matrix m_inverse;

private:
	typedef Eigen::Matrix<Matrix::Scalar, BlockSize, BlockSize, Matrix::Options> Block;

	/// Gets a lower-diagonal block element (named -Ai in the algorithm)
	/// \param A The matrix on which to retrieve the lower-diagonal block element
	/// \param i The line index on which to retrieve the lower-diagonal block element
	/// \return The lower-diagonal block element requested (i.e. block (i, i-1))
	const Eigen::Block<const Matrix, BlockSize, BlockSize> minusAi(const SurgSim::Math::Matrix& A, size_t i) const;

	/// Gets a diagonal block element (named Bi in the algorithm)
	/// \param A The matrix on which to retrieve the diagonal block element
	/// \param i The line index on which to retrieve the diagonal block element
	/// \return The diagonal block element requested (i.e. block (i, i))
	const Eigen::Block<const Matrix, BlockSize, BlockSize> Bi(const SurgSim::Math::Matrix& A, size_t i) const;

	/// Gets a upper-diagonal block element (named -Ci in the algorithm)
	/// \param A The matrix on which to retrieve the upper-diagonal block element
	/// \param i The line index on which to retrieve the upper-diagonal block element
	/// \return The upper-diagonal block element requested (i.e. block (i, i+1))
	const Eigen::Block<const Matrix, BlockSize, BlockSize> minusCi(const SurgSim::Math::Matrix& A, size_t i) const;

	///@{
	/// Intermediate block matrices, helpful to construct the inverse matrix
	std::vector<Block> m_Di, m_Ei, m_Bi_AiDiminus1_inv;
	///@}
};

/// Derivation for symmetric tri-diagonal block matrix type
/// \tparam BlockSize Define the block size of the tri-diagonal block matrix
template <size_t BlockSize>
class LinearSolveAndInverseSymmetricTriDiagonalBlockMatrix :
	public LinearSolveAndInverseTriDiagonalBlockMatrix<BlockSize>
{
public:
	void operator ()(const Matrix& A, const Vector& b, Vector* x = nullptr, Matrix* Ainv = nullptr) override;

	using LinearSolveAndInverseTriDiagonalBlockMatrix<BlockSize>::inverseTriDiagonalBlock;
	using LinearSolveAndInverseTriDiagonalBlockMatrix<BlockSize>::m_inverse;
};

}; // namespace Math

}; // namespace SurgSim

#include "SurgSim/Math/LinearSolveAndInverse-inl.h"

#endif // SURGSIM_MATH_LINEARSOLVEANDINVERSE_H

// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

/// \file
/// Definitions of small fixed-size square matrix types.

#ifndef SURGSIM_MATH_MATRIX_H
#define SURGSIM_MATH_MATRIX_H

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU> 		// needed for determinant() and inverse()
#include <Eigen/Sparse>

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace Math
{

/// A 2x2 matrix of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  2, 2, Eigen::RowMajor>  Matrix22f;

/// A 3x3 matrix of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  3, 3, Eigen::RowMajor>  Matrix33f;

/// A 4x4 matrix of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  4, 4, Eigen::RowMajor>  Matrix44f;

/// A 2x2 matrix of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 2, 2, Eigen::RowMajor>  Matrix22d;

/// A 3x3 matrix of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor>  Matrix33d;

/// A 4x4 matrix of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor>  Matrix44d;

/// A 6x6 matrix of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Matrix66d;

/// A dynamic size diagonal matrix
typedef Eigen::DiagonalMatrix<double, Eigen::Dynamic> DiagonalMatrix;

/// A dynamic size matrix
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;

/// A sparse matrix
typedef Eigen::SparseMatrix<double> SparseMatrix;

/// Create a rotation matrix corresponding to the specified angle (in radians) and axis.
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam VOpt the option flags (alignment etc.) used for the axis vector argument.  Can be deduced.
/// \param angle the angle of the rotation, in radians.
/// \param axis the axis of the rotation.
/// \returns the rotation matrix.
template <typename T, int VOpt>
inline Eigen::Matrix<T, 3, 3> makeRotationMatrix(const T& angle, const Eigen::Matrix<T, 3, 1, VOpt>& axis)
{
	return Eigen::AngleAxis<T>(angle, axis).toRotationMatrix();
}

/// Create a skew-symmetric matrix corresponding to the specified vector.  Skew-symmetric matrices are particularly
/// useful for representing a portion of the vector cross-product.
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam VOpt the option flags (alignment etc.) used for the vector argument.  Can be deduced.
/// \param vector the vector to be transformed.
/// \returns the skew-symmetric matrix corresponding with the vector argument.
template <typename T, int VOpt>
inline Eigen::Matrix<T, 3, 3> makeSkewSymmetricMatrix(const Eigen::Matrix<T, 3, 1, VOpt>& vector)
{
	Eigen::Matrix<T, 3, 3> result;

	result(0, 0) = 0.0;
	result(0, 1) = -vector(2);
	result(0, 2) = vector(1);

	result(1, 0) = vector(2);
	result(1, 1) = 0.0;
	result(1, 2) = -vector(0);

	result(2, 0) = -vector(1);
	result(2, 1) = vector(0);
	result(2, 2) = 0.0;

	return result;
}

/// Extract the unique vector from the skew-symmetric part of a given matrix.
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam MOpt the option flags (alignment etc.) used for the matrix argument.  Can be deduced.
/// \param matrix the matrix to compute the skew symmetric part from.
/// \returns the unique vector defining the skew-symmetric part of the matrix.
/// \note For any vector u, skew(makeSkewSymmetricMatrix(u)) = u
/// \note In general, returns the vector of the skew symmetric part of matrix: (matrix - matrix^T)/2
template <typename T, int MOpt>
inline Eigen::Matrix<T, 3, 1> skew(const Eigen::Matrix<T, 3, 3, MOpt>& matrix)
{
	Eigen::Matrix<T, 3, 3, MOpt> skewSymmetricPart = (matrix - matrix.transpose()) / 2.0;
	return Eigen::Matrix<T, 3, 1>(skewSymmetricPart(2, 1), skewSymmetricPart(0, 2), skewSymmetricPart(1, 0));
}

/// Get the angle (in radians) and axis corresponding to a rotation matrix.
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam MOpt the option flags (alignment etc.) used for the rotation matrix argument.  Can be deduced.
/// \tparam VOpt the option flags (alignment etc.) used for the axis vector argument.  Can be deduced.
/// \param matrix the rotation matrix to inspect.
/// \param [out] angle the angle of the rotation, in radians.
/// \param [out] axis the axis of the rotation.
template <typename T, int MOpt, int VOpt>
inline void computeAngleAndAxis(const Eigen::Matrix<T, 3, 3, MOpt>& matrix,
								T* angle, Eigen::Matrix<T, 3, 1, VOpt>* axis)
{
	Eigen::AngleAxis<T> angleAxis(matrix);
	*angle = angleAxis.angle();
	*axis = angleAxis.axis();
}

/// Get the angle corresponding to a quaternion's rotation, in radians.
/// If you don't care about the rotation axis, this is more efficient than computeAngleAndAxis().
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam MOpt the option flags (alignment etc.) used for the rotation matrix argument.  Can be deduced.
/// \param matrix the rotation matrix to inspect.
/// \returns the angle of the rotation, in radians.
template <typename T, int MOpt>
inline T computeAngle(const Eigen::Matrix<T, 3, 3, MOpt>& matrix)
{
	// TODO(bert): there has to be a more efficient way...
	Eigen::AngleAxis<T> angleAxis(matrix);
	return angleAxis.angle();
}

/// Helper method to add a sub-matrix into a matrix, for the sake of clarity
/// \tparam Matrix The matrix type
/// \tparam SubMatrix The sub-matrix type
/// \param subMatrix The sub-matrix
/// \param blockIdRow, blockIdCol The block indices in matrix
/// \param blockSizeRow, blockSizeCol The block size (size of the sub-matrix)
/// \param[out] matrix The matrix to add the sub-matrix into
template <class Matrix, class SubMatrix>
void addSubMatrix(const SubMatrix& subMatrix, size_t blockIdRow, size_t blockIdCol,
				  size_t blockSizeRow, size_t blockSizeCol, Matrix* matrix)
{
	matrix->block(blockSizeRow * blockIdRow, blockSizeCol * blockIdCol, blockSizeRow, blockSizeCol) += subMatrix;
}

/// Helper method to add a sub-matrix into a matrix, for the sake of clarity
/// \param subMatrix The sub-matrix
/// \param blockIdRow, blockIdCol The block indices in matrix
/// \param blockSizeRow, blockSizeCol The block size (size of the sub-matrix)
/// \param[out] matrix The matrix to add the sub-matrix into
/// \note This is a specialization of addSubMatrix for sparse matrices.
template <>
inline void addSubMatrix(const SparseMatrix& subMatrix, size_t blockIdRow, size_t blockIdCol,
						 size_t blockSizeRow, size_t blockSizeCol, SparseMatrix* matrix)
{
	typedef Eigen::Triplet<double> T;
	std::vector<T> tripletList;
	tripletList.reserve(36);

	SurgSim::Math::SparseMatrix dSparse(matrix->rows(), matrix->cols());

	for (size_t row = 0; row < blockSizeRow; ++row)
	{
		for (size_t col = 0; col < blockSizeCol; ++col)
		{
			tripletList.push_back(T((blockSizeRow * blockIdRow) + row, (blockSizeCol * blockIdCol) + col,
									subMatrix.coeff(row, col)));
		}
	}

	dSparse.setFromTriplets(tripletList.begin(), tripletList.end());
	*matrix += dSparse;
}


/// Helper method to add a sub-matrix into a matrix, for the sake of clarity
/// \param subMatrix The sub-matrix
/// \param blockIdRow, blockIdCol The block indices in matrix
/// \param blockSizeRow, blockSizeCol The block size (size of the sub-matrix)
/// \param[out] matrix The matrix to add the sub-matrix into
/// \note This is a specialization of addSubMatrix for sparse matrices with a dense submatrix.
template <>
inline void addSubMatrix(const Matrix& subMatrix, size_t blockIdRow, size_t blockIdCol,
						 size_t blockSizeRow, size_t blockSizeCol, SparseMatrix* matrix)
{
	typedef Eigen::Triplet<double> T;
	std::vector<T> tripletList;
	tripletList.reserve(36);

	SurgSim::Math::SparseMatrix dSparse(matrix->rows(), matrix->cols());

	for (size_t row = 0; row < blockSizeRow; ++row)
	{
		for (size_t col = 0; col < blockSizeCol; ++col)
		{
			tripletList.push_back(T((blockSizeRow * blockIdRow) + row, (blockSizeCol * blockIdCol) + col,
									subMatrix.coeff(row, col)));
		}
	}

	dSparse.setFromTriplets(tripletList.begin(), tripletList.end());
	*matrix += dSparse;
}

/// Helper method to add a sub-matrix made of squared-blocks into a matrix, for the sake of clarity
/// \tparam Matrix The matrix type
/// \tparam SubMatrix The sub-matrix type
/// \param subMatrix The sub-matrix (containing all the squared-blocks)
/// \param blockIds Vector of block indices (for accessing matrix) corresponding to the blocks in sub-matrix
/// \param blockSize The blocks size
/// \param[out] matrix The matrix to add the sub-matrix blocks into
template <class Matrix, class SubMatrix>
void addSubMatrix(const SubMatrix& subMatrix, const std::vector<size_t> blockIds, size_t blockSize, Matrix* matrix)
{
	const size_t numBlocks = blockIds.size();

	for (size_t block0 = 0; block0 < numBlocks; block0++)
	{
		size_t blockId0 = blockIds[block0];

		for (size_t block1 = 0; block1 < numBlocks; block1++)
		{
			size_t blockId1 = blockIds[block1];

			matrix->block(blockSize * blockId0, blockSize * blockId1, blockSize, blockSize) +=
				subMatrix.block(blockSize * block0, blockSize * block1, blockSize, blockSize);
		}
	}
}

/// Helper method to add a sub-matrix made of squared-blocks into a matrix, for the sake of clarity
/// \param subMatrix The sub-matrix (containing all the squared-blocks)
/// \param blockIds Vector of block indices (for accessing matrix) corresponding to the blocks in sub-matrix
/// \param blockSize The blocks size
/// \param[out] matrix The matrix to add the sub-matrix blocks into
/// \note This is a specialization of addSubMatrix for sparse matrices.
template <>
inline void addSubMatrix(const SparseMatrix& subMatrix, const std::vector<size_t> blockIds,
						 size_t blockSize, SparseMatrix* matrix)
{
	const size_t numBlocks = blockIds.size();

	typedef Eigen::Triplet<double> T;
	std::vector<T> tripletList;
	tripletList.reserve(36);

	SurgSim::Math::SparseMatrix dSparse(matrix->rows(), matrix->cols());

	for (size_t block0 = 0; block0 < numBlocks; block0++)
	{
		size_t blockId0 = blockIds[block0];

		for (size_t block1 = 0; block1 < numBlocks; block1++)
		{
			size_t blockId1 = blockIds[block1];

			for (int row = 0; row < blockSize; ++row)
			{
				for (int col = 0; col < blockSize; ++col)
				{
					tripletList.push_back(T(blockSize * blockId0 + row,  blockSize * blockId1 + col,
											subMatrix.coeff((blockSize * block0) + row, (blockSize * block1) + col)));
				}
			}
		}
	}
	dSparse.setFromTriplets(tripletList.begin(), tripletList.end());
	*matrix += dSparse;
}

/// Helper method to add a sub-matrix made of squared-blocks into a matrix, for the sake of clarity
/// \param subMatrix The sub-matrix (containing all the squared-blocks)
/// \param blockIds Vector of block indices (for accessing matrix) corresponding to the blocks in sub-matrix
/// \param blockSize The blocks size
/// \param[out] matrix The matrix to add the sub-matrix blocks into
/// \note This is a specialization of addSubMatrix for sparse matrices with a dense submatrix.
template <>
inline void addSubMatrix(const Matrix& subMatrix, const std::vector<size_t> blockIds,
						 size_t blockSize, SparseMatrix* matrix)
{
	const size_t numBlocks = blockIds.size();

	typedef Eigen::Triplet<double> T;
	std::vector<T> tripletList;
	tripletList.reserve(36);

	SurgSim::Math::SparseMatrix dSparse(matrix->rows(), matrix->cols());

	for (size_t block0 = 0; block0 < numBlocks; block0++)
	{
		size_t blockId0 = blockIds[block0];

		for (size_t block1 = 0; block1 < numBlocks; block1++)
		{
			size_t blockId1 = blockIds[block1];

			for (int row = 0; row < blockSize; ++row)
			{
				for (int col = 0; col < blockSize; ++col)
				{
					tripletList.push_back(T(blockSize * blockId0 + row,  blockSize * blockId1 + col,
											subMatrix.coeff((blockSize * block0) + row, (blockSize * block1) + col)));
				}
			}
		}
	}

	dSparse.setFromTriplets(tripletList.begin(), tripletList.end());
	*matrix += dSparse;
}
/// Helper method to add a sub-matrix made of squared-blocks into a matrix, for the sake of clarity
/// \param subMatrix The sub-matrix (containing all the squared-blocks)
/// \param blockIds Vector of block indices (for accessing matrix) corresponding to the blocks in sub-matrix
/// \param blockSize The blocks size
/// \param[out] matrix The matrix to add the sub-matrix blocks into
/// \note This is a specialization of addSubMatrix for sparse matrices with a dense submatrix.
template <>
inline void addSubMatrix(const Eigen::Matrix<double, 12, 12>& subMatrix, const std::vector<size_t> blockIds,
						 size_t blockSize, SparseMatrix* matrix)
{
	const size_t numBlocks = blockIds.size();

	typedef Eigen::Triplet<double> T;
	std::vector<T> tripletList;
	tripletList.reserve(36);

	SURGSIM_ASSERT((numBlocks * blockSize) == 12);
	SurgSim::Math::SparseMatrix dSparse(matrix->rows(), matrix->cols());

	for (size_t block0 = 0; block0 < numBlocks; block0++)
	{
		size_t blockId0 = blockIds[block0];

		for (size_t block1 = 0; block1 < numBlocks; block1++)
		{
			size_t blockId1 = blockIds[block1];

			for (int row = 0; row < blockSize; ++row)
			{
				for (int col = 0; col < blockSize; ++col)
				{
					tripletList.push_back(T(blockSize * blockId0 + row,  blockSize * blockId1 + col,
											subMatrix.coeff((blockSize * block0) + row, (blockSize * block1) + col)));
				}
			}
		}
	}

	dSparse.setFromTriplets(tripletList.begin(), tripletList.end());
	*matrix += dSparse;
}

/// Helper method to set a sub-matrix into a matrix, for the sake of clarity
/// \tparam Matrix The matrix type
/// \tparam SubMatrix The sub-matrix type
/// \param subMatrix The sub-matrix
/// \param blockIdRow, blockIdCol The block indices for row and column in matrix
/// \param blockSizeRow, blockSizeCol The size of the sub-matrix
/// \param[out] matrix The matrix to set the sub-matrix into
template <class Matrix, class SubMatrix>
void setSubMatrix(const SubMatrix& subMatrix, size_t blockIdRow, size_t blockIdCol,
				  size_t blockSizeRow, size_t blockSizeCol, Matrix* matrix)
{
	matrix->block(blockSizeRow * blockIdRow, blockSizeCol * blockIdCol,
				  blockSizeRow, blockSizeCol) = subMatrix;
}

/// Helper method to access a sub-matrix from a matrix, for the sake of clarity
/// \tparam Matrix The matrix type to get the sub-matrix from
/// \param matrix The matrix to get the sub-matrix from
/// \param blockIdRow, blockIdCol The block indices
/// \param blockSizeRow, blockSizeCol The block size
/// \return The requested sub-matrix
/// \note Disable cpplint warnings for use of non-const reference
/// \note Eigen has a specific type for Block that we want to return with read/write access
/// \note therefore the Matrix from which the Block is built from must not be const
template <class Matrix>
Eigen::Block<Matrix> getSubMatrix(Matrix& matrix, size_t blockIdRow, size_t blockIdCol,  // NOLINT
								  size_t blockSizeRow, size_t blockSizeCol)
{
	return matrix.block(blockSizeRow * blockIdRow, blockSizeCol * blockIdCol, blockSizeRow, blockSizeCol);
}

/// Helper method to zero a row of a matrix.
/// \tparam Matrix The matrix type
/// \param row The row to set to zero
/// \param[out] matrix The matrix to set the zero row on.
template <class Matrix>
void zeroRow(size_t row, Matrix* matrix)
{
	matrix->middleRows(row, 1).setZero();
}

/// Helper method to zero a row of a matrix specialized for Sparse Matrices
/// \param row The row to set to zero
/// \param[out] matrix The matrix to set the zero row on.
/// \note TODO: This can be made more efficient for either Compressed Sparse Row, or
/// \note Compressed Sparse Column by taking better advantage of the storage format.
/// \note Alternately, moving the boundary condition calculation so that it is determined
/// \note before the sparse matrix is generates could also improve performance.
template <>
inline void zeroRow(size_t row, SparseMatrix* matrix)
{
	for (int column = 0; column < matrix->cols(); ++column)
	{
		if (matrix->coeff(row, column))
		{
			matrix->coeffRef(row, column) = 0;
		}
	}
}

/// Helper method to zero a row of a matrix.
/// \tparam Matrix The matrix type
/// \param row The row to set to zero
/// \param[out] matrix The matrix to set the zero row on.
template <class Matrix>
void zeroColumn(size_t column, Matrix* matrix)
{
	(*matrix).middleCols(column, 1).setZero();
}

/// Helper method to zero a row of a matrix specialized for Sparse Matrices
/// \param row The row to set to zero
/// \param[out] matrix The matrix to set the zero row on.
/// \note TODO: This can be made more efficient for either Compressed Sparse Row, or
/// \note Compressed Sparse Column by taking better advantage of the storage format.
/// \note Alternately, moving the boundary condition calculation so that it is determined
/// \note before the sparse matrix is generates could also improve performance.
template <>
inline void zeroColumn(size_t column, SparseMatrix* matrix)
{
	for (int row = 0; row < matrix->rows(); ++row)
	{
		if (matrix->coeff(row, column))
		{
			matrix->coeffRef(row, column) = 0;
		}
	}
}

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_MATRIX_H

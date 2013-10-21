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

#include <SurgSim/Framework/Assert.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU> 		// needed for determinant() and inverse()

namespace SurgSim
{
namespace Math
{

/// A 2x2 matrix of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  2, 2, Eigen::DontAlign | Eigen::RowMajor>  Matrix22f;

/// A 3x3 matrix of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  3, 3, Eigen::DontAlign | Eigen::RowMajor>  Matrix33f;

/// A 4x4 matrix of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  4, 4, Eigen::DontAlign | Eigen::RowMajor>  Matrix44f;

/// A 2x2 matrix of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign | Eigen::RowMajor>  Matrix22d;

/// A 3x3 matrix of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 3, 3, Eigen::DontAlign | Eigen::RowMajor>  Matrix33d;

/// A 4x4 matrix of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign | Eigen::RowMajor>  Matrix44d;

/// A dynamic size diagonal matrix
typedef Eigen::DiagonalMatrix<double, Eigen::Dynamic> DiagonalMatrix;

/// A dynamic size matrix
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> Matrix;

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
void addSubMatrix(const SubMatrix& subMatrix, unsigned int blockIdRow, unsigned int blockIdCol,
	unsigned int blockSizeRow, unsigned int blockSizeCol, Matrix* matrix)
{
	matrix->block(blockSizeRow * blockIdRow, blockSizeCol * blockIdCol, blockSizeRow, blockSizeCol) += subMatrix;
}

/// Helper method to add a sub-matrix per block into a square matrix, for the sake of clarity
/// \tparam Matrix The matrix type
/// \tparam SubMatrix The sub-matrix type
/// \param subMatrix The sub-matrix (containing all the blocks)
/// \param blockIds Vector of block indices (for accessing matrix) corresponding to the blocks in sub-matrix
/// \param blockSize The block size (square)
/// \param[out] matrix The matrix to add the sub-matrix blocks into
template <class Matrix, class SubMatrix>
void addSubMatrix(const SubMatrix& subMatrix, const std::vector<unsigned int> blockIds,
	unsigned int blockSize, Matrix* matrix)
{
	const unsigned int springNumNodes = blockIds.size();

	for (unsigned int springNodeId0 = 0; springNodeId0 < springNumNodes; springNodeId0++)
	{
		unsigned int nodeId0 = blockIds[springNodeId0];

		for (unsigned int springNodeId1 = 0; springNodeId1 < springNumNodes; springNodeId1++)
		{
			unsigned int nodeId1 = blockIds[springNodeId1];

			matrix->block(blockSize * nodeId0, blockSize * nodeId1, blockSize, blockSize)
				+= subMatrix.block(blockSize * springNodeId0, blockSize * springNodeId1, blockSize, blockSize);
		}
	}
}

/// Helper method to set a sub-matrix into a matrix, for the sake of clarity
/// \tparam Matrix The matrix type
/// \tparam SubMatrix The sub-matrix type
/// \param subMatrix The sub-matrix
/// \param blockIdRow, blockIdCol The block indices for row and column in matrix
/// \param blockSizeRow, blockSizeCol The size of the sub-matrix
/// \param[out] matrix The matrix to set the sub-matrix into
template <class Matrix, class SubMatrix>
void setSubMatrix(const SubMatrix& subMatrix, unsigned int blockIdRow, unsigned int blockIdCol,
	unsigned int blockSizeRow, unsigned int blockSizeCol, Matrix* matrix)
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
template <class Matrix>
Eigen::Block<Matrix> getSubMatrix(Matrix& matrix, unsigned int blockIdRow, unsigned int blockIdCol,
	unsigned int blockSizeRow, unsigned int blockSizeCol)
{
	return matrix.block(blockSizeRow * blockIdRow, blockSizeCol * blockIdCol, blockSizeRow, blockSizeCol);
}

/// Helper methods to resize/allocate a matrix with a given size (if necessary), and potentially zero it out
/// \param[in,out] A matrix to resize
/// \param numRow, numCol The size to account for
/// \param zeroOut True if the vector v should be filled up with 0, False if not
/// \note This template method is useful to account for different matrix class having different API
template <class Matrix>
void resize(Matrix* A, unsigned int numRow, unsigned int numCol, bool zeroOut = false)
{
	if (! A)
	{
		return;
	}
	if (A->rows() != static_cast<int>(numRow) && A->cols() != static_cast<int>(numCol))
	{
		A->resize(static_cast<int>(numRow), static_cast<int>(numCol));
	}
	if (zeroOut)
	{
		A->setZero();
	}
}

template <>
void resize<DiagonalMatrix>(DiagonalMatrix* A, unsigned int numRow, unsigned int numCol, bool zeroOut);

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_MATRIX_H

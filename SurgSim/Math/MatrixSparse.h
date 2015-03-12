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

/// \file MatrixSparse.h
/// Definitions of useful sparse matrix functions

#ifndef SURGSIM_MATH_MATRIXSPARSE_H
#define SURGSIM_MATH_MATRIXSPARSE_H

#include <Eigen/Sparse>

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace Math
{

/// Set a SparseMatrix block<n, m>(i, j) from a (n x m) 'sub' matrix. <br>
/// It supposes: <br>
/// + that the SparseMatrix already contains all the elements within the block (no insertion necessary) <br>
/// + that both the SparseMatrix and the 'sub' matrix are using the same Scalar type <br>
/// + that the block in the SparseMatrix contains all the non-zero elements on these rows and columns <br>
/// This function will not change anything to the structure of the SparseMatrix, only change the values of the
/// corresponding coefficients.
/// \tparam n, m The block size (Derived may be bigger but cannot be smaller in both dimension)
/// \tparam performChecks Performs the necessary range checks to ensure no data violation if True
/// \tparam DerivedSub The type of the sub matrix (can usually be inferred). Can be any type, but does not
/// support Eigen expression. If it is a Sparse storage type the alignment must be the same as the SparseMatrix: Opt.
/// Note that no assertion or verification is done on this type.
/// \tparam T, Opt, I Types and option defining the output matrix type SparseMatrix<T, Opt, I>
/// \param subMatrix The 'sub' matrix that will be copied into the SparseMatrix block
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param[in,out] matrix The sparse matrix in which the block needs to be set by 'sub'
/// \exception SurgSim::Framework::AssertionFailure If performChecks is true and one of the condition is met: <br>
/// * if 'sub' is smaller than (n x m) in any dimension <br>
/// * if 'matrix' is nullptr or smaller than (n x m) in any dimension <br>
/// * if the requested block is out of range in 'matrix'. <br>
/// * if 'matrix' does not fulfill the requirement (i.e. has more elements on the block rows/columns than the block
///   itself, or is missing elements within the block).
/// \note The SparseMatrix 'matrix' must have a structure like the following: <br>
/// (xx 00 x) <br>
/// (xx 00 x) <br>
/// (00[xx]0) -> The block must already contain all the coefficients and no other coefficients should exist on <br>
/// (00[xx]0) -> these rows and columns. <br>
/// (xx 00 x) <br>
template <size_t n, size_t m, bool performChecks, typename DerivedSub, typename T, int Opt, typename I>
void setSubMatrixWithoutSearch(const DerivedSub& subMatrix,
							   I rowStart,
							   I columnStart,
							   Eigen::SparseMatrix<T, Opt, I>* matrix)
{
	typedef typename DerivedSub::Index ISub;

	static_assert(std::is_same<T, typename DerivedSub::Scalar>::value,
		"Both matrices should use the same Scalar type");

	if (performChecks)
	{
		SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

		SURGSIM_ASSERT(subMatrix.rows() >= static_cast<ISub>(n));
		SURGSIM_ASSERT(subMatrix.cols() >= static_cast<ISub>(m));

		SURGSIM_ASSERT(matrix->rows() >= static_cast<I>(n));
		SURGSIM_ASSERT(matrix->cols() >= static_cast<I>(m));
	}

	T* ptr = matrix->valuePtr();
	const I* innerIndices = matrix->innerIndexPtr();
	const I* outerIndices = matrix->outerIndexPtr();

	if (Opt == Eigen::ColMajor)
	{
		for (I colId = 0; colId < static_cast<I>(m); ++colId)
		{
			typedef Eigen::Matrix<T, n, 1, Eigen::DontAlign | Eigen::ColMajor> ColVector;

			// outerIndices[columnStart + colId] is the index of the 1st element in the column columnStart + colId in
			// both arrays ptr and innerIndices

			const I startIndexCurrentColumn = outerIndices[columnStart + colId];
			if (performChecks)
			{
				const I startIndexNextColumn = outerIndices[columnStart + colId + 1];

				// Make sure that we are not going to write out of the range...
				// i.e. The column has at least n elements
				SURGSIM_ASSERT(static_cast<I>(n) <= startIndexNextColumn - startIndexCurrentColumn);

				// Make sure that the 1st element in this column is the requested row
				SURGSIM_ASSERT(rowStart == innerIndices[startIndexCurrentColumn]);

				// Make sure that the last element corresponding to the block size is the expected row index
				SURGSIM_ASSERT(rowStart + static_cast<I>(n) - 1 == innerIndices[startIndexNextColumn - 1]);
			}

			// ptr[outerIndices[columnStart + colId]] is the 1st element in the column columnStart + colId
			// The n elements exist and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
			Eigen::Map<ColVector>(&ptr[startIndexCurrentColumn]).operator=(
				subMatrix.col(static_cast<ISub>(colId)).template segment<n>(0));
		}
	}
	else
	{
		for (I rowId = 0; rowId < static_cast<I>(n); ++rowId)
		{
			typedef Eigen::Matrix<T, 1, m, Eigen::DontAlign | Eigen::RowMajor> RowVector;

			// outerIndices[rowStart + rowId] is the index of the 1st element in the row rowStart + rowId in both
			// arrays ptr and innerIndices

			const I startIndexCurrentRow = outerIndices[rowStart + rowId];
			if (performChecks)
			{
				const I startIndexNextRow = outerIndices[rowStart + rowId + 1];

				// Make sure that we are not going to write out of the range...
				// i.e. The column has at least n elements
				SURGSIM_ASSERT(static_cast<I>(m) <= startIndexNextRow - startIndexCurrentRow);

				// Make sure that the 1st element in this row is the requested column
				SURGSIM_ASSERT(columnStart == innerIndices[startIndexCurrentRow]);

				// Make sure that the last element corresponding to the block size is the expected column index
				SURGSIM_ASSERT(columnStart + static_cast<I>(m) - 1 == innerIndices[startIndexNextRow - 1]);
			}

			// ptr[outerIndices[rowStart + rowId]] is the 1st element in the row rowStart + rowId
			// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
			Eigen::Map<RowVector>(&ptr[startIndexCurrentRow]).operator=(
				subMatrix.row(static_cast<ISub>(rowId)).template segment<m>(0));
		}
	}
}



/// Set a SparseMatrix block<n, m>(i, j) from a (n x m) 'sub' matrix. <br>
/// It supposes: <br>
/// + that the SparseMatrix already contains all the elements within the block (no insertion necessary) <br>
/// + that both the SparseMatrix and the 'sub' matrix are using the same Scalar type <br>
/// This function will not change anything to the structure of the SparseMatrix, only change the values of the
/// corresponding coefficients.
/// \tparam n, m The block size (Derived may be bigger but cannot be smaller in both dimension)
/// \tparam performChecks Performs the necessary range checks to ensure no data violation if True
/// \tparam DerivedSub The type of the sub matrix (can usually be inferred). Can be any type, but does not
/// support Eigen expression. If it is a Sparse storage type the alignment must be the same as the SparseMatrix: Opt.
/// Note that no assertion or verification is done on this type.
/// \tparam T, Opt, I Types and option defining the output matrix type SparseMatrix<T, Opt, I>
/// \param subMatrix The 'sub' matrix that will be copied into the SparseMatrix block
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param[in,out] matrix The sparse matrix in which the block needs to be set by 'sub'
/// \exception SurgSim::Framework::AssertionFailure If performChecks is true and one of the condition is met: <br>
/// * if 'sub' is smaller than (n x m) in any dimension <br>
/// * if 'matrix' is nullptr or smaller than (n x m) in any dimension <br>
/// * if the requested block is out of range in 'matrix'. <br>
/// * if 'matrix' does not fulfill the requirement (i.e. is missing elements within the block). <br>
/// \note The receiving SparseMatrix must have a structure like the following: <br>
/// (xx x0 x) <br>
/// (xx 0x x) <br>
/// (x0[xx]x) -> The block must already contain all the coefficients but these rows and columns may <br>
/// (0x[xx]0) -> contains more coefficients before and after the block. <br>
/// (xx 00 x) <br>
template <size_t n, size_t m, bool performChecks, typename DerivedSub, typename T, int Opt, typename I>
void setSubMatrixWithSearch(const DerivedSub& subMatrix,
							I rowStart,
							I columnStart,
							Eigen::SparseMatrix<T, Opt, I>* matrix)
{
	typedef typename DerivedSub::Index ISub;

	static_assert(std::is_same<T, typename DerivedSub::Scalar>::value,
		"Both matrices should use the same Scalar type");

	if (performChecks)
	{
		SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

		SURGSIM_ASSERT(subMatrix.rows() >= static_cast<ISub>(n));
		SURGSIM_ASSERT(subMatrix.cols() >= static_cast<ISub>(m));

		SURGSIM_ASSERT(matrix->rows() >= static_cast<I>(n));
		SURGSIM_ASSERT(matrix->cols() >= static_cast<I>(m));
	}

	T* ptr = matrix->valuePtr();
	const I* innerIndices = matrix->innerIndexPtr();
	const I* outerIndices = matrix->outerIndexPtr();

	if (Opt == Eigen::ColMajor)
	{
		for (I colId = 0; colId < static_cast<I>(m); ++colId)
		{
			typedef Eigen::Matrix<T, n, 1, Eigen::DontAlign | Eigen::ColMajor> ColVector;

			// outerIndices[columnStart + colId] is the index of the 1st element in the column columnStart + colId in
			/// both arrays ptr and innerIndices
			const I startIndexCurrentColumn = outerIndices[columnStart + colId];
			const I startIndexNextColumn = outerIndices[columnStart + colId + 1];

			// Special case of an empty column
			if (startIndexNextColumn == startIndexCurrentColumn)
			{
				continue;
			}

			// Look for the index of rowStart in this column (the column may contain elements before)
			I indexFirstRow;
			if (innerIndices[startIndexCurrentColumn] == rowStart)
			{
				indexFirstRow = startIndexCurrentColumn;
			}
			else
			{
				indexFirstRow = matrix->data().searchLowerIndex(
					startIndexCurrentColumn, startIndexNextColumn - 1, rowStart);
			}

			if (performChecks)
			{
				// Make sure we actually found the element (rowStart, columnStart + colId) in matrix
				SURGSIM_ASSERT(innerIndices[indexFirstRow] == rowStart);

				// Make sure that we are not going to write out of the range...
				// i.e. The column (starting at the beginning of the block) has at least n elements
				SURGSIM_ASSERT(static_cast<I>(n) <= startIndexNextColumn - indexFirstRow);

				// Make sure that the last element corresponding to the block size is the expected row index
				SURGSIM_ASSERT(rowStart + static_cast<I>(n) - 1 == \
					innerIndices[indexFirstRow + static_cast<I>(n) - 1]);
			}

			// ptr[outerIndices[columnStart + colId]] is the 1st element in the column columnStart + colId
			// ptr[indexFirstRow] is the 1st element in the column within the requested block
			// The n elements exist and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
			Eigen::Map<ColVector>(&ptr[indexFirstRow]).operator=(
				subMatrix.col(static_cast<ISub>(colId)).template segment<n>(0));
		}
	}
	else
	{
		for (I rowId = 0; rowId < static_cast<I>(n); ++rowId)
		{
			typedef Eigen::Matrix<T, 1, m, Eigen::DontAlign | Eigen::RowMajor> RowVector;

			// outerIndices[rowStart + rowId] is the index of the 1st element in the row rowStart + rowId in both
			// arrays ptr and innerIndices
			const I startIndexCurrentRow = outerIndices[rowStart + rowId];
			const I startIndexNextRow = outerIndices[rowStart + rowId + 1];

			// Special case of an empty row
			if (startIndexNextRow == startIndexCurrentRow)
			{
				continue;
			}

			// Look for the index of columnStart in this row (the row may contain elements before)
			I indexFirstColumn;
			if (innerIndices[startIndexCurrentRow] == columnStart)
			{
				indexFirstColumn = startIndexCurrentRow;
			}
			else
			{
				indexFirstColumn = matrix->data().searchLowerIndex(
					startIndexCurrentRow, startIndexNextRow - 1, columnStart);
			}

			if (performChecks)
			{
				// Make sure we actually found the element (rowStart + rowId, columnStart) in matrix
				SURGSIM_ASSERT(innerIndices[indexFirstColumn] == columnStart);

				// Make sure that we are not going to write out of the range...
				// i.e. The row (starting at the beginning of the block) has at least m elements
				SURGSIM_ASSERT(static_cast<I>(m) <= startIndexNextRow - indexFirstColumn);

				// Make sure that the last element corresponding to the block size is the expected column index
				SURGSIM_ASSERT(columnStart + static_cast<I>(m) - 1 == \
					innerIndices[indexFirstColumn + static_cast<I>(m) - 1]);
			}

			// ptr[outerIndices[rowStart + rowId]] is the 1st element in the row rowStart + rowId
			// ptr[indexFirstColumn] is the 1st element in the row within the requested block
			// The m elements exist and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
			Eigen::Map<RowVector>(&ptr[indexFirstColumn]).operator=(
				subMatrix.row(static_cast<ISub>(rowId)).template segment<m>(0));
		}
	}
}

/// Assign a SparseMatrix/SparseVector/Sparse expression to a SparseMatrix block
/// \tparam DerivedSub Type of the sub matrix/vector (any SparseMatrix, SparseVector or sparse expression)
/// \tparam T, Opt, I Types and option defining the output matrix type SparseMatrix<T, Opt, I>
/// \param subMatrix The sub-matrix/vector to set
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param[in,out] matrix The sparse matrix in which the block needs to be set by 'sub'
/// \exception SurgSim::Framework::AssertionFailure If 'matrix' is a nullptr or the block is out of 'matrix' range
/// \note The size of the block to set is directly given by subMatrix size. <br>
/// \note No assumption is made on any matrix/vector, it executes a slow insertion/search. <br>
/// \note If the structure of the matrix is known and constant through time, it is recommended to
/// \note pre-allocate the matrix structure and use an optimized dedicated setXXX method.
template <typename DerivedSub, typename T, int Opt, typename I>
void setSparseMatrixBlock(const Eigen::SparseMatrixBase<DerivedSub>& subMatrix,
						  I rowStart,
						  I columnStart,
						  Eigen::SparseMatrix<T, Opt, I>* matrix)
{
	typedef typename DerivedSub::InnerIterator InnerIterator;

	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	SURGSIM_ASSERT(matrix->rows() >= rowStart + subMatrix.rows());
	SURGSIM_ASSERT(matrix->cols() >= columnStart + subMatrix.cols());

	for (auto outer = 0; outer < subMatrix.outerSize(); outer++)
	{
		for (InnerIterator it(subMatrix.const_cast_derived(), outer); it; ++it)
		{
			matrix->coeffRef(rowStart + static_cast<I>(it.row()), columnStart + static_cast<I>(it.col())) =
				static_cast<T>(it.value());
		}
	}
}

/// Add a SparseMatrix/SparseVector/Sparse expression to a SparseMatrix block
/// \tparam DerivedSub Type of the sub matrix/vector (any SparseMatrix, SparseVector or sparse expression)
/// \tparam T, Opt, I Types and option defining the output matrix type SparseMatrix<T, Opt, I>
/// \param subMatrix The sub-matrix/vector to add to the output
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param[in,out] matrix The sparse matrix in which the block needs to be incremented by 'sub'
/// \exception SurgSim::Framework::AssertionFailure If 'matrix' is a nullptr or the block is out of 'matrix' range
/// \note The size of the block to set is directly given by subMatrix size. <br>
/// \note No assumption is made on any matrix/vector, it executes a slow insertion/search. <br>
template <typename DerivedSub, typename T, int Opt, typename I>
void addSparseMatrixBlock(const Eigen::SparseMatrixBase<DerivedSub>& subMatrix,
						  I rowStart,
						  I columnStart,
						  Eigen::SparseMatrix<T, Opt, I>* matrix)
{
	typedef typename DerivedSub::InnerIterator InnerIterator;

	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	SURGSIM_ASSERT(matrix->rows() >= rowStart + subMatrix.rows());
	SURGSIM_ASSERT(matrix->cols() >= columnStart + subMatrix.cols());

	for (auto outer = 0; outer < subMatrix.outerSize(); outer++)
	{
		for (InnerIterator it(subMatrix.const_cast_derived(), outer); it; ++it)
		{
			matrix->coeffRef(rowStart + static_cast<I>(it.row()), columnStart + static_cast<I>(it.col())) +=
				static_cast<T>(it.value());
		}
	}
}

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_MATRIXSPARSE_H

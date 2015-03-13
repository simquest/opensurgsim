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

/// \file SparseMatrix.h
/// Definitions of useful sparse matrix functions

#ifndef SURGSIM_MATH_SPARSEMATRIX_H
#define SURGSIM_MATH_SPARSEMATRIX_H

#include <Eigen/Sparse>

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace Math
{

namespace
{
/// Helper class to assign a column/row of a matrix to a chunk of memory
/// \tparam T The type of data in the chunk of memory
/// \tparam Opt The option of the matrix/vector alignment
/// \tparam Index The index type to access the chunk of memory
/// \tparam n, m The number of elements to treat (n for ColMajor, m for RowMajor)
/// \tparam DerivedSub The class of the matrix from which the data are copied
template <class T, int Opt, class Index, int n, int m, class DerivedSub>
class op
{
public:
	/// Do the assignment of a row/column of a matrix to a chunk of memory
	/// \param ptr The chunk of memory
	/// \param start Where the assignment starts in the chunk of memory
	/// \param subMatrix The matrix from which the row/column is copied
	/// \param colRowId The column or row id depending on the template parameter Opt
	void assign(T* ptr, Index start, const DerivedSub& subMatrix, Index colRowId){}
};

/// Specialization for column major storage
template <class T, class Index, int n, int m, class DerivedSub>
class op<T, Eigen::ColMajor, Index, n, m, DerivedSub>
{
public:
	void assign(T* ptr, Index start, const DerivedSub& subMatrix, Index colId)
	{
		typedef Eigen::Matrix<T, n, 1, Eigen::DontAlign | Eigen::ColMajor> ColVector;

		// ptr[start] is the 1st element in the column
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
		Eigen::Map<ColVector>(&ptr[start]).operator=(
			subMatrix.col(static_cast<typename DerivedSub::Index>(colId)).template segment<n>(0));
	}
};

/// Specialization for row major storage
template <class T, class Index, int n, int m, class DerivedSub>
class op<T, Eigen::RowMajor, Index, n, m, DerivedSub>
{
public:
	void assign(T* ptr, Index start, const DerivedSub& subMatrix, Index rowId)
	{
		typedef Eigen::Matrix<T, 1, m, Eigen::DontAlign | Eigen::RowMajor> RowVector;

		// ptr[start] is the 1st element in the row
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
		Eigen::Map<RowVector>(&ptr[start]).operator=(
			subMatrix.row(static_cast<typename DerivedSub::Index>(rowId)).template segment<m>(0));
	}
};
}

/// Set a SparseMatrix block<n, m>(i, j) from a (n x m) 'subMatrix' without searching for the block elements.<br>
/// It supposes: <br>
/// + that the SparseMatrix already contains all the elements within the block (no insertion necessary) <br>
/// + that both the SparseMatrix and the 'subMatrix' are using the same Scalar type <br>
/// + that the block in the SparseMatrix contains all the non-zero elements on these rows and columns <br>
/// This function will not change anything to the structure of the SparseMatrix, only change the values of the
/// corresponding coefficients.
/// \tparam n, m The block size (Derived may be bigger but cannot be smaller in both dimension)
/// \tparam DerivedSub The type of the 'subMatrix' (can usually be inferred). Can be any type, but does not
/// support Eigen expression. If it is a Sparse storage type the alignment must be the same as the SparseMatrix: Opt.
/// Note that no assertion or verification is done on this type.
/// \tparam T, Opt, Index Types and option defining the output matrix type SparseMatrix<T, Opt, Index>
/// \param subMatrix The sub matrix that will be copied into the SparseMatrix block
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param[in,out] matrix The sparse matrix in which the block needs to be set by 'subMatrix'
/// \exception SurgSim::Framework::AssertionFailure If one of the following conditions is met: <br>
/// * if 'subMatrix' is smaller than (n x m) in any dimension <br>
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
template <size_t n, size_t m, bool performChecks, typename DerivedSub, typename T, int Opt, typename Index>
void setSubMatrixWithoutSearch(const DerivedSub& subMatrix,
							   Index rowStart,
							   Index columnStart,
							   Eigen::SparseMatrix<T, Opt, Index>* matrix)
{
	typedef typename DerivedSub::Index DerivedSubIndexType;

	static_assert(std::is_same<T, typename DerivedSub::Scalar>::value,
		"Both matrices should use the same Scalar type");

	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	SURGSIM_ASSERT(subMatrix.rows() >= static_cast<DerivedSubIndexType>(n)) << "subMatrix doesn't have enough rows";
	SURGSIM_ASSERT(subMatrix.cols() >= static_cast<DerivedSubIndexType>(m)) << "subMatrix doesn't have enough columns";

	SURGSIM_ASSERT(matrix->rows() >= rowStart + static_cast<Index>(n)) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->cols() >= columnStart + static_cast<Index>(m)) << "The block is out of range in matrix";

	T* ptr = matrix->valuePtr();
	const Index* innerIndices = matrix->innerIndexPtr();
	const Index* outerIndices = matrix->outerIndexPtr();

	if (Opt == Eigen::ColMajor)
	{
		for (Index colId = 0; colId < static_cast<Index>(m); ++colId)
		{
			// outerIndices[columnStart + colId] is the index in ptr and innerIndices of the first non-zero element in
			// the column (columnStart + colId)
			const Index currentColumnRowStart = outerIndices[columnStart + colId];
			const Index nextColumnRowStart = outerIndices[columnStart + colId + 1];

			// Make sure that we are not going to write out of the range...
			// i.e. The column has at least n elements
			SURGSIM_ASSERT(static_cast<Index>(n) <= nextColumnRowStart - currentColumnRowStart) <<
				"matrix column " << colId << " doesn't have enough coefficients";

			// Make sure that the 1st element in this column is the requested row
			SURGSIM_ASSERT(rowStart == innerIndices[currentColumnRowStart]) <<
				"matrix column " << colId << " doesn't start at the block start location";

			// Make sure that the last element corresponding to the block size is the expected row index
			SURGSIM_ASSERT(rowStart + static_cast<Index>(n) - 1 == innerIndices[nextColumnRowStart - 1]) <<
				"matrix column " << colId << " doesn't end at the block end location";

			op<T, Opt, Index, n, m, DerivedSub>().assign(ptr, currentColumnRowStart, subMatrix, colId);
		}
	}
	else
	{
		for (Index rowId = 0; rowId < static_cast<Index>(n); ++rowId)
		{
			// outerIndices[rowStart + rowId] is the index in ptr and innerIndices of the first non-zero element in
			// the row (rowStart + rowId)
			const Index currentRowColumnStart = outerIndices[rowStart + rowId];
			const Index nextRowColumnStart = outerIndices[rowStart + rowId + 1];

			// Make sure that we are not going to write out of the range...
			// i.e. The column has at least n elements
			SURGSIM_ASSERT(static_cast<Index>(m) <= nextRowColumnStart - currentRowColumnStart) <<
				"matrix row " << rowId << " doesn't have enough coefficients";

			// Make sure that the 1st element in this row is the requested column
			SURGSIM_ASSERT(columnStart == innerIndices[currentRowColumnStart]) <<
				"matrix row " << rowId << " doesn't start at the block start location";

			// Make sure that the last element corresponding to the block size is the expected column index
			SURGSIM_ASSERT(columnStart + static_cast<Index>(m) - 1 == innerIndices[nextRowColumnStart - 1]) <<
				"matrix row " << rowId << " doesn't end at the block end location";

			op<T, Opt, Index, n, m, DerivedSub>().assign(ptr, currentRowColumnStart, subMatrix, rowId);
		}
	}
}



/// Set a SparseMatrix block<n, m>(i, j) from a (n x m) 'subMatrix', searching for the block 1st element. <br>
/// It supposes: <br>
/// + that the SparseMatrix already contains all the elements within the block (no insertion necessary) <br>
/// + that both the SparseMatrix and the 'subMatrix' are using the same Scalar type <br>
/// This function will not change anything to the structure of the SparseMatrix, only change the values of the
/// corresponding coefficients.
/// \tparam n, m The block size (Derived may be bigger but cannot be smaller in both dimension)
/// \tparam DerivedSub The type of the 'subMatrix' (can usually be inferred). Can be any type, but does not
/// support Eigen expression. If it is a Sparse storage type the alignment must be the same as the SparseMatrix: Opt.
/// Note that no assertion or verification is done on this type.
/// \tparam T, Opt, Index Types and option defining the output matrix type SparseMatrix<T, Opt, Index>
/// \param subMatrix The sub matrix that will be copied into the SparseMatrix block
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param[in,out] matrix The sparse matrix in which the block needs to be set by 'subMatrix'
/// \exception SurgSim::Framework::AssertionFailure If one of the following conditions is met: <br>
/// * if 'subMatrix' is smaller than (n x m) in any dimension <br>
/// * if 'matrix' is nullptr or smaller than (n x m) in any dimension <br>
/// * if the requested block is out of range in 'matrix'. <br>
/// * if 'matrix' does not fulfill the requirement (i.e. is missing elements within the block). <br>
/// \note The receiving SparseMatrix must have a structure like the following: <br>
/// (xx x0 x) <br>
/// (xx 0x x) <br>
/// (x0[xx]x) -> The block must already contain all the coefficients but these rows and columns may <br>
/// (0x[xx]0) -> contains more coefficients before and after the block. <br>
/// (xx 00 x) <br>
template <size_t n, size_t m, bool performChecks, typename DerivedSub, typename T, int Opt, typename Index>
void setSubMatrixWithSearch(const DerivedSub& subMatrix,
							Index rowStart,
							Index columnStart,
							Eigen::SparseMatrix<T, Opt, Index>* matrix)
{
	typedef typename DerivedSub::Index DerivedSubIndexType;

	static_assert(std::is_same<T, typename DerivedSub::Scalar>::value,
		"Both matrices should use the same Scalar type");

	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	SURGSIM_ASSERT(subMatrix.rows() >= static_cast<DerivedSubIndexType>(n)) << "subMatrix doesn't have enough rows";
	SURGSIM_ASSERT(subMatrix.cols() >= static_cast<DerivedSubIndexType>(m)) << "subMatrix doesn't have enough columns";

	SURGSIM_ASSERT(matrix->rows() >= rowStart + static_cast<Index>(n)) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->cols() >= columnStart + static_cast<Index>(m)) << "The block is out of range in matrix";

	T* ptr = matrix->valuePtr();
	const Index* innerIndices = matrix->innerIndexPtr();
	const Index* outerIndices = matrix->outerIndexPtr();

	if (Opt == Eigen::ColMajor)
	{
		for (Index colId = 0; colId < static_cast<Index>(m); ++colId)
		{
			// outerIndices[columnStart + colId] is the index in ptr and innerIndices of the first non-zero element in
			// the column (columnStart + colId)
			const Index currentColumnRowStart = outerIndices[columnStart + colId];
			const Index nextColumnRowStart = outerIndices[columnStart + colId + 1];

			// Look for the index of rowStart in this column (the column may contain elements before)
			Index indexFirstRow;
			if (innerIndices[currentColumnRowStart] == rowStart)
			{
				indexFirstRow = currentColumnRowStart;
			}
			else
			{
				indexFirstRow = matrix->data().searchLowerIndex(
					currentColumnRowStart, nextColumnRowStart - 1, rowStart);
			}

			// Make sure we actually found the element (rowStart, columnStart + colId) in matrix
			SURGSIM_ASSERT(innerIndices[indexFirstRow] == rowStart) <<
				"matrix (" << rowStart << "," << columnStart + colId << ") could not be located";

			// Make sure that we are not going to write out of the range...
			// i.e. The column (starting at the beginning of the block) has at least n elements
			SURGSIM_ASSERT(static_cast<Index>(n) <= nextColumnRowStart - indexFirstRow) <<
				"matrix column " << colId << " doesn't have enough coefficients";

			// Make sure that the last element corresponding to the block size is the expected row index
			SURGSIM_ASSERT(rowStart + static_cast<Index>(n) - 1 == \
				innerIndices[indexFirstRow + static_cast<Index>(n) - 1]) <<
				"matrix column " << colId << " is missing elements in the block";

			op<T, Opt, Index, n, m, DerivedSub>().assign(ptr, indexFirstRow, subMatrix, colId);
		}
	}
	else
	{
		for (Index rowId = 0; rowId < static_cast<Index>(n); ++rowId)
		{
			// outerIndices[rowStart + rowId] is the index in ptr and innerIndices of the first non-zero element in
			// the column (rowStart + rowId)
			const Index currentRowColumnStart = outerIndices[rowStart + rowId];
			const Index nextRowColumnStart = outerIndices[rowStart + rowId + 1];

			// Look for the index of columnStart in this row (the row may contain elements before)
			Index indexFirstColumn;
			if (innerIndices[currentRowColumnStart] == columnStart)
			{
				indexFirstColumn = currentRowColumnStart;
			}
			else
			{
				indexFirstColumn = matrix->data().searchLowerIndex(
					currentRowColumnStart, nextRowColumnStart - 1, columnStart);
			}

			// Make sure we actually found the element (rowStart + rowId, columnStart) in matrix
			SURGSIM_ASSERT(innerIndices[indexFirstColumn] == columnStart) <<
				"matrix (" << rowStart + rowId << "," << columnStart << ") could not be located";

			// Make sure that we are not going to write out of the range...
			// i.e. The row (starting at the beginning of the block) has at least m elements
			SURGSIM_ASSERT(static_cast<Index>(m) <= nextRowColumnStart - indexFirstColumn) <<
				"matrix row " << rowId << " doesn't have enough coefficients";

			// Make sure that the last element corresponding to the block size is the expected column index
			SURGSIM_ASSERT(columnStart + static_cast<Index>(m) - 1 == \
				innerIndices[indexFirstColumn + static_cast<Index>(m) - 1]) <<
				"matrix row " << rowId << " is missing elements in the block";

			op<T, Opt, Index, n, m, DerivedSub>().assign(ptr, indexFirstColumn, subMatrix, rowId);
		}
	}
}

/// Assign a SparseMatrix/SparseVector/Sparse expression to a SparseMatrix block
/// \tparam DerivedSub Type of the sub matrix/vector (any SparseMatrix, SparseVector or sparse expression)
/// \tparam T, Opt, Index Types and option defining the output matrix type SparseMatrix<T, Opt, Index>
/// \param subMatrix The sub-matrix/vector to set
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param[in,out] matrix The sparse matrix in which the block needs to be set by 'subMatrix'
/// \exception SurgSim::Framework::AssertionFailure If 'matrix' is a nullptr or the block is out of 'matrix' range
/// \note The size of the block to set is directly given by 'subMatrix' size. <br>
/// \note No assumption is made on any matrix/vector, it executes a slow insertion/search. <br>
/// \note If the structure of the matrix is known and constant through time, it is recommended to
/// \note pre-allocate the matrix structure and use an optimized dedicated setXXX method.
template <typename DerivedSub, typename T, int Opt, typename Index>
void setSparseMatrixBlock(const Eigen::SparseMatrixBase<DerivedSub>& subMatrix,
						  Index rowStart,
						  Index columnStart,
						  Eigen::SparseMatrix<T, Opt, Index>* matrix)
{
	typedef typename DerivedSub::InnerIterator InnerIterator;

	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	SURGSIM_ASSERT(matrix->rows() >= rowStart + subMatrix.rows()) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->cols() >= columnStart + subMatrix.cols()) << "The block is out of range in matrix";

	for (auto outer = 0; outer < subMatrix.outerSize(); outer++)
	{
		for (InnerIterator it(subMatrix.const_cast_derived(), outer); it; ++it)
		{
			matrix->coeffRef(rowStart + static_cast<Index>(it.row()), columnStart + static_cast<Index>(it.col())) =
				static_cast<T>(it.value());
		}
	}
}

/// Add a SparseMatrix/SparseVector/Sparse expression to a SparseMatrix block
/// \tparam DerivedSub Type of the sub matrix/vector (any SparseMatrix, SparseVector or sparse expression)
/// \tparam T, Opt, Index Types and option defining the output matrix type SparseMatrix<T, Opt, Index>
/// \param subMatrix The sub-matrix/vector to add to the output
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param[in,out] matrix The sparse matrix in which the block needs to be incremented by 'subMatrix'
/// \exception SurgSim::Framework::AssertionFailure If 'matrix' is a nullptr or the block is out of 'matrix' range
/// \note The size of the block to set is directly given by subMatrix size. <br>
/// \note No assumption is made on any matrix/vector, it executes a slow insertion/search. <br>
template <typename DerivedSub, typename T, int Opt, typename Index>
void addSparseMatrixBlock(const Eigen::SparseMatrixBase<DerivedSub>& subMatrix,
						  Index rowStart,
						  Index columnStart,
						  Eigen::SparseMatrix<T, Opt, Index>* matrix)
{
	typedef typename DerivedSub::InnerIterator InnerIterator;

	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	SURGSIM_ASSERT(matrix->rows() >= rowStart + subMatrix.rows()) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->cols() >= columnStart + subMatrix.cols()) << "The block is out of range in matrix";

	for (auto outer = 0; outer < subMatrix.outerSize(); outer++)
	{
		for (InnerIterator it(subMatrix.const_cast_derived(), outer); it; ++it)
		{
			matrix->coeffRef(rowStart + static_cast<Index>(it.row()), columnStart + static_cast<Index>(it.col())) +=
				static_cast<T>(it.value());
		}
	}
}

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_SPARSEMATRIX_H

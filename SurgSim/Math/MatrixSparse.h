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
/// Definitions of sparse matrix type and useful functions

#ifndef SURGSIM_MATH_MATRIXSPARSE_H
#define SURGSIM_MATH_MATRIXSPARSE_H

#include <Eigen/Sparse>

namespace SurgSim
{
namespace Math
{

/// This method set the submatrix M<n, m>(i, j) of a SparseMatrix from a (n x m) 'sub' matrix.
/// It supposes: <br>
/// + that the SparseMatrix already contains all the elements within the block (no insertion necessary)
/// + that both the SparseMatrix and the 'sub' matrix are using the same Scalar type.
/// + that the corresponding block in the SparseMatrix contains all the non-zero elements on these rows and columns
///   i.e. no search needed to locate the beginning of the block.
/// This function will not change anything to the structure of the SparseMatrix, only change the values of the
/// corresponding coefficients.
/// \tparam Derived The Derived class of the sub matrix (can usually be inferred)
/// \tparam n, m The block size (Derived may be bigger but cannot be smaller in both dimension)
/// \tparam performChecks Performs the necessary range checks to ensure no data violation if True
/// \tparam T The scalar type for the sparse matrix
/// \tparam Opt The option parameter for the sparse matrix (Eigen::ColMajor|Eigen::RowMajor}
/// \param sub The matrix that will be copied into the SparseMatrix block
/// \param[in,out] S The sparse matrix to be set
/// \param i, j The row and column indices to indicate where the block to be set starts in S
/// \exception SurgSim::Framework::AssertionFailure if sub is smaller than (n x m) in any dimension
/// \note If Derived is a SparseMatrix, it must have the same alignment option as S (i.e. Opt)
/// \note The receiving SparseMatrix must have a structure like the following:
/// \note The receiving SparseMatrix must have a structure like the following:
/// (xx 00 x)
/// (xx 00 x)
/// (00[xx]0) -> The block must already contain all the coefficients and no other coefficients should exist on
/// (00[xx]0)    these rows and columns.
/// (xx 00 x)
template <typename Derived, size_t n, size_t m, bool performChecks, class T, int Opt>
void setSubMatrixWithoutSearch(const Derived& sub,
							   Eigen::SparseMatrix<T, Opt>* S,
							   typename Eigen::SparseMatrix<T, Opt>::Index i,
							   typename Eigen::SparseMatrix<T, Opt>::Index j)
{
	typedef typename Eigen::SparseMatrix<T, Opt>::Index Index;
	typedef typename Eigen::SparseMatrix<T, Opt>::Scalar Scalar;

	static_assert(std::is_same<Scalar, T>::value, "Both matrices should use the same Scalar type");

	SURGSIM_ASSERT(sub.rows() >= n);
	SURGSIM_ASSERT(sub.cols() >= m);

	Scalar* ptr = S->valuePtr();
	const Index* innerIndices = S->innerIndexPtr();
	const Index* outerIndices = S->outerIndexPtr();

	if (Opt == Eigen::ColMajor)
	{
		for (Index colId = 0; colId < m; ++colId)
		{
			typedef Eigen::Matrix<T, n, 1, Eigen::DontAlign | Eigen::ColMajor> ColVector;

			// outerIndices[j + colId] is the index of the 1st element in the column j+colId in both arrays
			// ptr and innerIndices

			const Index startIndexCurrentColumn = outerIndices[j + colId];
			if (performChecks)
			{
				const Index startIndexNextColumn = outerIndices[j + colId + 1];

				// Make sure that we are not going to write out of the range...
				// i.e. The column has at least n elements
				SURGSIM_ASSERT(n <= startIndexNextColumn - startIndexCurrentColumn);

				// Make sure that the 1st element in this column is the requested row
				SURGSIM_ASSERT(i == innerIndices[startIndexCurrentColumn]);

				// Make sure that the last element corresponding to the block size is the expected row index
				SURGSIM_ASSERT(i + n - 1 == innerIndices[startIndexNextColumn - 1]);
			}

			// ptr[outerIndices[j + colId]] is the 1st element in the column j+colId
			// The n elements exist and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
			Eigen::Map<ColVector>(&ptr[startIndexCurrentColumn]).operator=(sub.col(colId).segment<n>(0));
		}
	}
	else
	{
		for (Index rowId = 0; rowId < n; ++rowId)
		{
			typedef Eigen::Matrix<T, 1, m, Eigen::DontAlign | Eigen::RowMajor> RowVector;

			// outerIndices[i + rowId] is the index of the 1st element in the row i+rowId in both arrays
			// ptr and innerIndices

			const Index startIndexCurrentRow = outerIndices[i + rowId];
			if (performChecks)
			{
				const Index startIndexNextRow = outerIndices[i + rowId + 1];

				// Make sure that we are not going to write out of the range...
				// i.e. The column has at least n elements
				SURGSIM_ASSERT(m <= startIndexNextRow - startIndexCurrentRow);

				// Make sure that the 1st element in this row is the requested column
				SURGSIM_ASSERT(j == innerIndices[startIndexCurrentRow]);

				// Make sure that the last element corresponding to the block size is the expected column index
				SURGSIM_ASSERT(j + m - 1 == innerIndices[startIndexNextRow - 1]);
			}

			// ptr[outerIndices[i + rowId]] is the 1st element in the row j+colId
			// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
			Eigen::Map<RowVector>(&ptr[startIndexCurrentRow]).operator=(sub.row(rowId).segment<m>(0));
		}
	}
}

/// This method set the submatrix M<n, m>(i, j) of a SparseMatrix from a (n x m) 'sub' matrix.
/// It supposes: <br>
/// + that the SparseMatrix already contains all the elements within the block (no insertion necessary)
/// + that both the SparseMatrix and the 'sub' matrix are using the same Scalar type.
/// - the SparseMatrix may have more elements on these rows and columns; i.e. a search needs to be performed to locate
///   the beginning of the block.
/// This function will not change anything to the structure of the SparseMatrix, only change the values of the
/// corresponding coefficients.
/// \tparam Derived The Derived class of the sub matrix (can usually be inferred)
/// \tparam n, m The block size (Derived may be bigger but cannot be smaller in both dimension)
/// \tparam T The scalar type for the sparse matrix
/// \tparam Opt The option parameter for the sparse matrix (Eigen::ColMajor|Eigen::RowMajor}
/// \param sub The matrix that will be copied into the SparseMatrix block
/// \param[in,out] S The sparse matrix to be set
/// \param i, j The row and column indices to indicate where the block to be set starts in S
/// \exception SurgSim::Framework::AssertionFailure if sub is smaller than (n x m) in any dimension
/// \note If Derived is a SparseMatrix, it must have the same alignment option as S (i.e. Opt)
/// \note The receiving SparseMatrix must have a structure like the following:
/// (xx x0 x)
/// (xx 0x x)
/// (x0[xx]x) -> The block must already contain all the coefficients but these rows and columns may
/// (0x[xx]0)    contains more coefficients before and after the block.
/// (xx 00 x)
template <typename Derived, size_t n, size_t m, bool performChecks, typename Derived2>
void setSubMatrixWithSearch(const Derived& subMatrix,
							Eigen::SparseMatrixBase<Derived2>* matrix,
							typename Derived2::Index i,
							typename Derived2::Index j)
{
	typedef typename Derived2::Index Index;
	typedef typename Derived2::Scalar Scalar;
	const int Opt = Derived::Options;

	static_assert(std::is_same<Scalar, typename Derived::T>::value, "Both matrices should use the same Scalar type");

	SURGSIM_ASSERT(subMatrix.rows() >= n);
	SURGSIM_ASSERT(subMatrix.cols() >= m);

	Scalar* ptr = matrix->valuePtr();
	const Index* innerIndices = matrix->innerIndexPtr();
	const Index* outerIndices = matrix->outerIndexPtr();

	if (Opt == Eigen::ColMajor)
	{
		for (Index colId = 0; colId < m; ++colId)
		{
			typedef Eigen::Matrix<T, n, 1, Eigen::DontAlign | Eigen::ColMajor> ColVector;

			// outerIndices[j + colId] is the index of the 1st element in the column j+colId in both arrays
			// ptr and innerIndices
			const Index startIndexCurrentColumn = outerIndices[j + colId];
			const Index startIndexNextColumn = outerIndices[j + colId + 1];

			// Special case of an empty column
			if (startIndexNextColumn == startIndexCurrentColumn)
			{
				continue;
			}

			// Look for the index of the row i in this column (the column may contain elements before)
			Index indexFirstRow;
			if (innerIndices[startIndexCurrentColumn] == i)
			{
				indexFirstRow = startIndexCurrentColumn;
			}
			else
			{
				indexFirstRow = S->data().searchLowerIndex(startIndexCurrentColumn, startIndexNextColumn - 1, i);
			}

			if (performChecks)
			{
				// Make sure that we are not going to write out of the range...
				// i.e. The column (starting at the beginning of the block) has at least n elements
				SURGSIM_ASSERT(n <= startIndexNextColumn - indexFirstRow);

				// Make sure that the last element corresponding to the block size is the expected row index
				SURGSIM_ASSERT(i + n - 1 == innerIndices[indexFirstRow + n - 1]);
			}

			// ptr[outerIndices[j + colId]] is the 1st element in the column j+colId
			// ptr[indexFirstRow] is the 1st element in the column within the requested block
			// The n elements exist and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
			Eigen::Map<ColVector>(&ptr[indexFirstRow]).operator=(subMatrix.col(colId).segment<n>(0));
		}
	}
	else
	{
		for (Index rowId = 0; rowId < n; ++rowId)
		{
			typedef Eigen::Matrix<T, 1, m, Eigen::DontAlign | Eigen::RowMajor> RowVector;

			// outerIndices[i + rowId] is the index of the 1st element in the row i+rowId in both arrays
			// ptr and innerIndices
			const Index startIndexCurrentRow = outerIndices[i + rowId];
			const Index startIndexNextRow = outerIndices[i + rowId + 1];

			// Special case of an empty row
			if (startIndexNextRow == startIndexCurrentRow)
			{
				continue;
			}

			// Look for the index of the column j in this row (the row may contain elements before)
			Index indexFirstColumn;
			if (innerIndices[startIndexCurrentRow] == j)
			{
				indexFirstColumn = startIndexCurrentRow;
			}
			else
			{
				indexFirstColumn = S->data().searchLowerIndex(startIndexCurrentRow, startIndexNextRow - 1, j);
			}

			if (performChecks)
			{
				// Make sure that we are not going to write out of the range...
				// i.e. The row (starting at the beginning of the block) has at least m elements
				SURGSIM_ASSERT(m <= startIndexNextRow - indexFirstColumn);

				// Make sure that the last element corresponding to the block size is the expected column index
				SURGSIM_ASSERT(j + m - 1 == innerIndices[indexFirstColumn + m - 1]);
			}

			// ptr[outerIndices[i + rowId]] is the 1st element in the row i+rowId
			// ptr[indexFirstColumn] is the 1st element in the row within the requested block
			// The m elements exist and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
			Eigen::Map<ColVector>(&ptr[indexFirstColumn]).operator=(subMatrix.row(rowId).segment<m>(0));
		}
	}
}

/// Set a SparseMatrix block using a SparseMatrix/SparseVector
/// \tparam Derived Any SparseMatrix or SparseVector
/// \tparam Derived2 Any SparseMatrix or SparseVector
/// \param subMatrix The sub-matrix/vector to set
/// \param matrix The SparseMatrix/SparseVector for which a block is set
/// \param i, j (row, column) indices of the block start in matrix to be set with subMatrix
/// \note The size of the block to set is directly given by subMatrix size
/// \note No assumption is made on any matrix/vector. Slow insertion/coefficient search algorithm.
/// \note If the structure of the matrix is known and constant through time, it is recommended to
/// \note pre-allocate the matrix structure and use an optimized dedicated setXXX method.
template <typename Derived, typename Derived2>
void setSparseMatrix(const Eigen::SparseMatrixBase<Derived>& subMatrix,
					 Eigen::SparseMatrixBase<Derived2>* matrix,
					 typename Derived2::Index i,
					 typename Derived2::Index j)
{
	typedef typename Derived2::Index Index;

	for (auto outer = 0; outer < subMatrix.outerSize(); outer++)
	{
		for (Derived::InnerIterator it(subMatrix, outer); it; ++it)
		{
			matrix->coeffRef(i + static_cast<Index>(it.row()), j + static_cast<Index>(it.col())) =
				static_cast<typename Derived2::Scalar>(it.value());
		}
	}
}

/// Add a SparseMatrix block using a SparseMatrix/SparseVector
/// \tparam Derived Any SparseMatrix or SparseVector
/// \tparam Derived2 Any SparseMatrix or SparseVector
/// \param subMatrix The sub-matrix/vector to set
/// \param matrix The SparseMatrix/SparseVector for which a block is set
/// \param i, j (row, column) indices of the block start in matrix to be set with subMatrix
/// \note The size of the block to set is directly given by subMatrix size
/// \note No assumption is made on any matrix/vector. Slow insertion/coefficient search algorithm.
/// \note If the structure of the matrix is known and constant through time, it is recommended to
/// \note pre-allocate the matrix structure and use an optimized dedicated setXXX method.
template <typename Derived, typename Derived2>
void addSparseMatrix(const Eigen::SparseMatrixBase<Derived>& subMatrix,
					 Eigen::SparseMatrixBase<Derived2>* matrix,
					 typename Derived2::Index i,
					 typename Derived2::Index j)
{
	typedef typename Derived2::Index Index;

	for (auto outer = 0; outer < subMatrix.outerSize(); outer++)
	{
		for (Derived::InnerIterator it(subMatrix, outer); it; ++it)
		{
			matrix->coeffRef(i + static_cast<Index>(it.row()), j + static_cast<Index>(it.col())) +=
				static_cast<typename Derived2::Scalar>(it.value());
		}
	}
}

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_MATRIXSPARSE_H

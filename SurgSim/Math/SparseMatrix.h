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

namespace Static
{
/// Helper class to run operation a column/row of a matrix to a chunk of memory where the size is statically defined
/// \tparam T The type of data in the chunk of memory
/// \tparam Opt The option of the matrix/vector alignment
/// \tparam Index The index type to access the chunk of memory
/// \tparam n, m The number of elements to treat (n for ColMajor, m for RowMajor)
/// \tparam DerivedSub The class of the matrix from which the data are copied
template <class T, int Opt, class Index, int n, int m, class DerivedSub>
class Operation
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
class Operation<T, Eigen::ColMajor, Index, n, m, DerivedSub>
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
class Operation<T, Eigen::RowMajor, Index, n, m, DerivedSub>
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

namespace Dynamic
{
/// Helper class to run operation a column/row of a matrix to a chunk of memory where the size is dynamically defined
/// \tparam T The type of data in the chunk of memory
/// \tparam Opt The option of the matrix/vector alignment
/// \tparam Index The index type to access the chunk of memory
/// \tparam DerivedSub The class of the matrix from which the data are copied
template <class T, int Opt, class Index, class DerivedSub>
class Operation
{
public:
	/// Do the assignment of a row/column of a matrix to a chunk of memory
	/// \param ptr The chunk of memory
	/// \param start Where the assignment starts in the chunk of memory
	/// \param n, m The size of the block (n rows, m columns)
	/// \param subMatrix The matrix from which the row/column is copied
	/// \param colRowId The column or row id depending on the template parameter Opt
	void assign(T* ptr, Index start, Index n, Index m, const DerivedSub& subMatrix, Index colRowId){}
};

/// Specialization for column major storage
template <class T, class Index, class DerivedSub>
class Operation<T, Eigen::ColMajor, Index, DerivedSub>
{
public:
	void assign(T* ptr, Index start, Index n, Index m, const DerivedSub& subMatrix, Index colId)
	{
		typedef Eigen::Matrix<T, Eigen::Dynamic, 1, Eigen::DontAlign | Eigen::ColMajor> ColVector;

		// ptr[start] is the 1st element in the column
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
		Eigen::Map<ColVector>(&ptr[start], n).operator=(
			subMatrix.col(static_cast<typename DerivedSub::Index>(colId)).segment(0, n));
	}
};

/// Specialization for row major storage
template <class T, class Index, class DerivedSub>
class Operation<T, Eigen::RowMajor, Index, DerivedSub>
{
public:
	void assign(T* ptr, Index start, Index n, Index m, const DerivedSub& subMatrix, Index rowId)
	{
		typedef Eigen::Matrix<T, 1, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor> RowVector;

		// ptr[start] is the 1st element in the row
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the copy
		Eigen::Map<RowVector>(&ptr[start], m).operator=(
			subMatrix.row(static_cast<typename DerivedSub::Index>(rowId)).segment(0, m));
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
/// support Eigen expression. If it is a Sparse storage type, the storage (RowMajor or ColMajor) must be the same as
/// in the SparseMatrix: Opt. Note that no assertion or verification is done on this type.
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
template <size_t n, size_t m, typename DerivedSub, typename T, int Opt, typename Index>
void setSubMatrixWithoutSearch(const DerivedSub& subMatrix,
							   Index rowStart,
							   Index columnStart,
							   Eigen::SparseMatrix<T, Opt, Index>* matrix)
{
	typedef typename DerivedSub::Index DerivedSubIndexType;

	static Static::Operation<T, Opt, Index, n, m, DerivedSub> operation;

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

	Index outerStart = (Opt == Eigen::ColMajor ? columnStart : rowStart);
	Index innerStart = (Opt == Eigen::ColMajor ? rowStart: columnStart);
	Index outerSize = static_cast<Index>(Opt == Eigen::ColMajor ? m : n);
	Index innerSize = static_cast<Index>(Opt == Eigen::ColMajor ? n : m);

	for (Index outerLoop = 0; outerLoop < outerSize; ++outerLoop)
	{
		// outerIndices[outerStart + outerLoop] is the index in ptr and innerIndices of the first non-zero element in
		// the outer element (outerStart + outerLoop)
		const Index innerStartIdInCurrentOuter = outerIndices[outerStart + outerLoop];
		const Index innerStartIdInNextOuter = outerIndices[outerStart + outerLoop + 1];

		// Make sure that we are not going to write out of the range...
		// i.e. The column has at least n elements
		SURGSIM_ASSERT(static_cast<Index>(innerSize) <= innerStartIdInNextOuter - innerStartIdInCurrentOuter) <<
			"matrix column/row " << outerStart + outerLoop << " doesn't have enough coefficients";

		// Make sure that the 1st element in this column is the requested row
		SURGSIM_ASSERT(innerStart == innerIndices[innerStartIdInCurrentOuter]) <<
			"matrix column/row " << outerStart + outerLoop << " doesn't start at the block start location";

		// Make sure that the last element corresponding to the block size is the expected row index
		SURGSIM_ASSERT(innerStart + static_cast<Index>(innerSize) - 1 == innerIndices[innerStartIdInNextOuter - 1]) <<
			"matrix column/row " << outerStart + outerLoop << " doesn't end at the block end location";

		operation.assign(ptr, innerStartIdInCurrentOuter, subMatrix, outerLoop);
	}
}

/// Set a SparseMatrix block(i, j, n, m) from a (n x m) 'subMatrix' without searching for the block elements.<br>
/// It supposes: <br>
/// + that the SparseMatrix already contains all the elements within the block (no insertion necessary) <br>
/// + that both the SparseMatrix and the 'subMatrix' are using the same Scalar type <br>
/// + that the block in the SparseMatrix contains all the non-zero elements on these rows and columns <br>
/// This function will not change anything to the structure of the SparseMatrix, only change the values of the
/// corresponding coefficients.
/// \tparam DerivedSub The type of the 'subMatrix' (can usually be inferred). Can be any type, but does not
/// support Eigen expression. If it is a Sparse storage type the alignment must be the same as the SparseMatrix: Opt.
/// Note that no assertion or verification is done on this type.
/// \tparam T, Opt, Index Types and option defining the output matrix type SparseMatrix<T, Opt, Index>
/// \param subMatrix The sub matrix that will be copied into the SparseMatrix block
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param n, m The block size, respectively number of rows and columns
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
template <typename DerivedSub, typename T, int Opt, typename Index>
void setSubMatrixWithoutSearch(const DerivedSub& subMatrix,
							   Index rowStart, Index columnStart,
							   Index n, Index m,
							   Eigen::SparseMatrix<T, Opt, Index>* matrix)
{
	typedef typename DerivedSub::Index DerivedSubIndexType;

	static Dynamic::Operation<T, Opt, Index, DerivedSub> operation;

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

	Index outerStart = (Opt == Eigen::ColMajor ? columnStart : rowStart);
	Index innerStart = (Opt == Eigen::ColMajor ? rowStart: columnStart);
	Index outerSize = static_cast<Index>(Opt == Eigen::ColMajor ? m : n);
	Index innerSize = static_cast<Index>(Opt == Eigen::ColMajor ? n : m);

	for (Index outerLoop = 0; outerLoop < outerSize; ++outerLoop)
	{
		// outerIndices[outerStart + outerLoop] is the index in ptr and innerIndices of the first non-zero element in
		// the outer element (outerStart + outerLoop)
		const Index innerStartIdInCurrentOuter = outerIndices[outerStart + outerLoop];
		const Index innerStartIdInNextOuter = outerIndices[outerStart + outerLoop + 1];

		// Make sure that we are not going to write out of the range...
		// i.e. The column has at least n elements
		SURGSIM_ASSERT(static_cast<Index>(innerSize) <= innerStartIdInNextOuter - innerStartIdInCurrentOuter) <<
			"matrix column/row " << outerStart + outerLoop << " doesn't have enough coefficients";

		// Make sure that the 1st element in this column is the requested row
		SURGSIM_ASSERT(innerStart == innerIndices[innerStartIdInCurrentOuter]) <<
			"matrix column/row " << outerStart + outerLoop << " doesn't start at the block start location";

		// Make sure that the last element corresponding to the block size is the expected row index
		SURGSIM_ASSERT(innerStart + static_cast<Index>(innerSize) - 1 == innerIndices[innerStartIdInNextOuter - 1]) <<
			"matrix column/row " << outerStart + outerLoop << " doesn't end at the block end location";

		operation.assign(ptr, innerStartIdInCurrentOuter, n, m, subMatrix, outerLoop);
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
template <size_t n, size_t m, typename DerivedSub, typename T, int Opt, typename Index>
void setSubMatrixWithSearch(const DerivedSub& subMatrix,
							Index rowStart,
							Index columnStart,
							Eigen::SparseMatrix<T, Opt, Index>* matrix)
{
	typedef typename DerivedSub::Index DerivedSubIndexType;

	static Static::Operation<T, Opt, Index, n, m, DerivedSub> operation;

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

	Index outerStart = (Opt == Eigen::ColMajor ? columnStart : rowStart);
	Index innerStart = (Opt == Eigen::ColMajor ? rowStart: columnStart);
	Index outerSize = static_cast<Index>(Opt == Eigen::ColMajor ? m : n);
	Index innerSize = static_cast<Index>(Opt == Eigen::ColMajor ? n : m);

	for (Index outerLoop = 0; outerLoop < outerSize; ++outerLoop)
	{
		// outerIndices[outerStart + outerLoop] is the index in ptr and innerIndices of the first non-zero element in
		// the outer element (outerStart + outerLoop)
		const Index innerStartIdInCurrentOuter = outerIndices[outerStart + outerLoop];
		const Index innerStartIdInNextOuter = outerIndices[outerStart + outerLoop + 1];

		// Look for the index of innerStart in this outer (the column/row may contain elements before)
		Index innerFirstElement;
		if (innerIndices[innerStartIdInCurrentOuter] == innerStart)
		{
			innerFirstElement = innerStartIdInCurrentOuter;
		}
		else
		{
			innerFirstElement = matrix->data().searchLowerIndex(
				innerStartIdInCurrentOuter, innerStartIdInNextOuter - 1, innerStart);
		}

		// Make sure we actually found the 1st element of the block in this outer
		SURGSIM_ASSERT(innerIndices[innerFirstElement] == innerStart) <<
			"matrix is missing an element of the block (1st element on a row/column)";

		// Make sure that we are not going to write out of the range...
		// i.e. The column/row (starting at the beginning of the block) has at least innerSize elements
		SURGSIM_ASSERT(static_cast<Index>(innerSize) <= innerStartIdInNextOuter - innerFirstElement) <<
			"matrix is missing elements of the block (but not the 1st element on a row/column)";

		// Make sure that the last element corresponding to the block size has the expected index
		SURGSIM_ASSERT(innerStart + static_cast<Index>(innerSize) - 1 == \
			innerIndices[innerFirstElement + static_cast<Index>(innerSize) - 1]) <<
			"matrix is missing elements of the block (but not the 1st element on a row/column)";

		operation.assign(ptr, innerFirstElement, subMatrix, outerLoop);
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

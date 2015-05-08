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
/// A sparse matrix
typedef Eigen::SparseMatrix<double> SparseMatrix;

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
	void assign(T* ptr, Index start, Index n, Index m, const DerivedSub& subMatrix, Index colRowId) {}

	/// Do the assignment of a single matrix element (operator =)
	/// \param ptr The matrix element to be assigned
	/// \param value The value to assign to
	void assign(T* ptr, const T& value) {}

	/// Do the addition of a row/column of a matrix to a chunk of memory
	/// \param ptr The chunk of memory
	/// \param start Where the addition starts in the chunk of memory
	/// \param n, m The size of the block (n rows, m columns)
	/// \param subMatrix The matrix from which the row/column is added
	/// \param colRowId The column or row id depending on the template parameter Opt
	void add(T* ptr, Index start, Index n, Index m, const DerivedSub& subMatrix, Index colRowId) {}

	/// Do the addition of a single matrix element (operator +=)
	/// \param ptr The matrix element to be increased
	/// \param value The value to add
	void add(T* ptr, const T& value) {}
};

/// Specialization for column major storage
template <class T, class Index, class DerivedSub>
class Operation<T, Eigen::ColMajor, Index, DerivedSub>
{
public:
	void assign(T* ptr, Index start, Index n, Index m, const DerivedSub& subMatrix, Index colId)
	{
		typedef Eigen::Matrix < T, Eigen::Dynamic, 1, Eigen::DontAlign | Eigen::ColMajor > ColVector;
		typedef typename Eigen::Matrix < T, Eigen::Dynamic, 1, Eigen::DontAlign | Eigen::ColMajor >::Index IndexVector;
		typedef typename DerivedSub::Index IndexSub;

		// ptr[start] is the 1st element in the column
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the operation
		Eigen::Map<ColVector>(&ptr[start], static_cast<IndexVector>(n)).operator = (
					subMatrix.col(static_cast<IndexSub>(colId)).segment(0, static_cast<IndexSub>(n)));
	}

	void assign(T* ptr, const T& value)
	{
		*ptr = value;
	}

	void add(T* ptr, Index start, Index n, Index m, const DerivedSub& subMatrix, Index colId)
	{
		typedef Eigen::Matrix < T, Eigen::Dynamic, 1, Eigen::DontAlign | Eigen::ColMajor > ColVector;
		typedef typename Eigen::Matrix < T, Eigen::Dynamic, 1, Eigen::DontAlign | Eigen::ColMajor >::Index IndexVector;
		typedef typename DerivedSub::Index IndexSub;

		// ptr[start] is the 1st element in the column
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the operation
		Eigen::Map<ColVector>(&ptr[start], static_cast<IndexVector>(n)).operator += (
					subMatrix.col(static_cast<IndexSub>(colId)).segment(0, static_cast<IndexSub>(n)));
	}

	void add(T* ptr, const T& value)
	{
		*ptr += value;
	}
};

/// Specialization for row major storage
template <class T, class Index, class DerivedSub>
class Operation<T, Eigen::RowMajor, Index, DerivedSub>
{
public:
	void assign(T* ptr, Index start, Index n, Index m, const DerivedSub& subMatrix, Index rowId)
	{
		typedef Eigen::Matrix < T, 1, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor > RowVector;
		typedef typename Eigen::Matrix < T, 1, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor >::Index IndexVector;
		typedef typename DerivedSub::Index IndexSub;

		// ptr[start] is the 1st element in the row
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the operation
		Eigen::Map<RowVector>(&ptr[start], static_cast<IndexVector>(m)).operator = (
					subMatrix.row(static_cast<IndexSub>(rowId)).segment(0, static_cast<IndexSub>(m)));
	}

	void assign(T* ptr, const T& value)
	{
		*ptr = value;
	}

	void add(T* ptr, Index start, Index n, Index m, const DerivedSub& subMatrix, Index rowId)
	{
		typedef Eigen::Matrix < T, 1, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor > RowVector;
		typedef typename Eigen::Matrix < T, 1, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor >::Index IndexVector;
		typedef typename DerivedSub::Index IndexSub;

		// ptr[start] is the 1st element in the row
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the operation
		Eigen::Map<RowVector>(&ptr[start], static_cast<IndexVector>(m)).operator += (
					subMatrix.row(static_cast<IndexSub>(rowId)).segment(0, static_cast<IndexSub>(m)));
	}

	void add(T* ptr, const T& value)
	{
		*ptr += value;
	}
};

/// Runs a given operation on a SparseMatrix block(i, j, n, m) from a (n x m) 'subMatrix' without searching for the
/// block elements.<br>
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
/// \param op The operation to run on the block
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
void blockWithoutSearch(const DerivedSub& subMatrix, Index rowStart, Index columnStart, Index n, Index m,
						Eigen::SparseMatrix<T, Opt, Index>* matrix,
						void (Operation<T, Opt, Index, DerivedSub>::*op)(T*, Index, Index, Index,
								const DerivedSub&, Index))
{
	typedef typename DerivedSub::Index DerivedSubIndexType;

	static Operation<T, Opt, Index, DerivedSub> operation;

	static_assert(std::is_same<T, typename DerivedSub::Scalar>::value,
				  "Both matrices should use the same Scalar type");

	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	SURGSIM_ASSERT(subMatrix.rows() >= static_cast<DerivedSubIndexType>(n)) << "subMatrix doesn't have enough rows";
	SURGSIM_ASSERT(subMatrix.cols() >= static_cast<DerivedSubIndexType>(m)) << "subMatrix doesn't have enough columns";

	SURGSIM_ASSERT(matrix->rows() >= rowStart + static_cast<Index>(n)) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->cols() >= columnStart + static_cast<Index>(m)) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->valuePtr() != nullptr) << "The matrix is not initialized correctly, null pointer to values";

	T* ptr = matrix->valuePtr();
	const Index* innerIndices = matrix->innerIndexPtr();
	const Index* outerIndices = matrix->outerIndexPtr();

	Index outerStart = (Opt == Eigen::ColMajor ? columnStart : rowStart);
	Index innerStart = (Opt == Eigen::ColMajor ? rowStart : columnStart);
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

		(operation.*op)(ptr, innerStartIdInCurrentOuter, n, m, subMatrix, outerLoop);
	}
}

/// Runs a given operation on a SparseMatrix block(i, j, n, m) from a (n x m) 'subMatrix' with searching for the
/// block 1st element. <br>
/// It supposes: <br>
/// + that the SparseMatrix already contains all the elements within the block (no insertion necessary) <br>
/// + that both the SparseMatrix and the 'subMatrix' are using the same Scalar type <br>
/// This function will not change anything to the structure of the SparseMatrix, only change the values of the
/// corresponding coefficients.
/// \tparam DerivedSub The type of the 'subMatrix' (can usually be inferred). Can be any type, but does not
/// support Eigen expression. If it is a Sparse storage type the alignment must be the same as the SparseMatrix: Opt.
/// Note that no assertion or verification is done on this type.
/// \tparam T, Opt, Index Types and option defining the output matrix type SparseMatrix<T, Opt, Index>
/// \param subMatrix The sub matrix that will be copied into the SparseMatrix block
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param n, m The block size (Derived may be bigger but cannot be smaller in both dimension)
/// \param[in,out] matrix The sparse matrix in which the block needs to be set by 'subMatrix'
/// \param op The operation to run on the block
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
template <typename DerivedSub, typename T, int Opt, typename Index>
void blockWithSearch(const DerivedSub& subMatrix, Index rowStart, Index columnStart, Index n, Index m,
					 Eigen::SparseMatrix<T, Opt, Index>* matrix,
					 void (Operation<T, Opt, Index, DerivedSub>::*op)(T*, Index, Index, Index,
							 const DerivedSub&, Index))
{
	typedef typename DerivedSub::Index DerivedSubIndexType;

	static Operation<T, Opt, Index, DerivedSub> operation;

	static_assert(std::is_same<T, typename DerivedSub::Scalar>::value,
				  "Both matrices should use the same Scalar type");

	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	SURGSIM_ASSERT(subMatrix.rows() >= static_cast<DerivedSubIndexType>(n)) << "subMatrix doesn't have enough rows";
	SURGSIM_ASSERT(subMatrix.cols() >= static_cast<DerivedSubIndexType>(m)) << "subMatrix doesn't have enough columns";

	SURGSIM_ASSERT(matrix->rows() >= rowStart + static_cast<Index>(n)) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->cols() >= columnStart + static_cast<Index>(m)) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->valuePtr() != nullptr) << "The matrix is not initialized correctly, null pointer to values";

	T* ptr = matrix->valuePtr();
	const Index* innerIndices = matrix->innerIndexPtr();
	const Index* outerIndices = matrix->outerIndexPtr();

	Index outerStart = (Opt == Eigen::ColMajor ? columnStart : rowStart);
	Index innerStart = (Opt == Eigen::ColMajor ? rowStart : columnStart);
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
							   "The last element of the block does not have the expected index. " <<
							   "The matrix is missing elements of the block (but not the 1st element on a row/column)";

		(operation.*op)(ptr, innerFirstElement, n, m, subMatrix, outerLoop);
	}
}

/// Runs a given operation on a SparseMatrix block using a SparseMatrix/SparseVector/Sparse expression as input
/// \tparam DerivedSub Type of the sub matrix/vector (any SparseMatrix, SparseVector or sparse expression)
/// \tparam T, Opt, Index Types and option defining the output matrix type SparseMatrix<T, Opt, Index>
/// \param subMatrix The sub-matrix/vector to use as input
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param[in,out] matrix The sparse matrix in which the block will be altered.
/// \param op The operation to run on the block
/// \exception SurgSim::Framework::AssertionFailure If 'matrix' is a nullptr or the block is out of 'matrix' range
/// \note The size of the block is directly given by 'subMatrix' size. <br>
/// \note No assumption is made on any matrix/vector, it executes a slow insertion/search. <br>
/// \note If the structure of the matrix is known and constant through time, it is recommended to
/// \note pre-allocate the matrix structure and use an optimized dedicated blockXXX method.
template <typename DerivedSub, typename T, int Opt, typename Index>
void blockOperation(const Eigen::SparseMatrixBase<DerivedSub>& subMatrix, Index rowStart, Index columnStart,
					Eigen::SparseMatrix<T, Opt, Index>* matrix,
					void (Operation<T, Opt, Index, DerivedSub>::*op)(T*, const T&))
{
	typedef typename DerivedSub::InnerIterator InnerIterator;

	static Operation<T, Opt, Index, DerivedSub> operation;

	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	SURGSIM_ASSERT(matrix->rows() >= rowStart + subMatrix.rows()) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->cols() >= columnStart + subMatrix.cols()) << "The block is out of range in matrix";

	for (auto outer = 0; outer < subMatrix.outerSize(); outer++)
	{
		for (InnerIterator it(subMatrix.const_cast_derived(), outer); it; ++it)
		{
			(operation.*op)(
				&matrix->coeffRef(rowStart + static_cast<Index>(it.row()), columnStart + static_cast<Index>(it.col())),
				static_cast<T>(it.value()));
		}
	}
}

/// Runs a given operation on a SparseMatrix block(i, j, n, m) from a (n x m) 'subMatrix' with searching for the
/// block 1st element. <br>
/// It supposes: <br>
/// + that both the SparseMatrix and the 'subMatrix' are using the same Scalar type <br>
/// This function can be used to initialize the form of a sparse matrix as it will add entries when appropriate
/// entries do not exist.
/// \tparam DerivedSub The type of the 'subMatrix' (can usually be inferred). Can be any type, but does not
/// support Eigen expression. If it is a Sparse storage type the alignment must be the same as the SparseMatrix: Opt.
/// Note that no assertion or verification is done on this type.
/// \tparam T, Opt, Index Types and option defining the output matrix type SparseMatrix<T, Opt, Index>
/// \param subMatrix The sub matrix that will be copied into the SparseMatrix block
/// \param rowStart, columnStart The row and column indices to indicate where the block in the SparseMatrix starts
/// \param[in,out] matrix The sparse matrix in which the block needs to be set by 'subMatrix'
/// \param op The operation to run on the block
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
template <typename DerivedSub, typename T, int Opt, typename Index>
void blockOperation(const DerivedSub& subMatrix, Index rowStart, Index columnStart,
					Eigen::SparseMatrix<T, Opt, Index>* matrix,
					void (Operation<T, Opt, Index, DerivedSub>::*op)(T*, const T&))
{
	static Operation<T, Opt, Index, DerivedSub> operation;

	static_assert(std::is_same<T, typename DerivedSub::Scalar>::value,
				  "Both matrices should use the same Scalar type");

	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	Index n = static_cast<Index>(subMatrix.rows());
	Index m = static_cast<Index>(subMatrix.cols());
	SURGSIM_ASSERT(matrix->rows() >= rowStart + static_cast<Index>(n)) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->cols() >= columnStart + static_cast<Index>(m)) << "The block is out of range in matrix";

	for (Index row = 0; row < n; ++row)
	{
		for (Index column = 0; column < m; ++column)
		{
			(operation.*op)(
				&matrix->coeffRef(rowStart + row, columnStart + column),
				static_cast<T>(subMatrix(row, column)));
		}
	}
}

/// Helper method to add a sub-matrix into a matrix, for the sake of clarity
/// \tparam DerivedSub The type of the 'subMatrix' (can usually be inferred). Can be any type, but does not
/// support Eigen expression. If it is a Sparse storage type the alignment must be the same as the SparseMatrix: Opt.
/// Note that no assertion or verification is done on this type.
/// \tparam T, Opt, Index Types and option defining the output matrix type SparseMatrix<T, Opt, Index>
/// \param subMatrix The sub-matrix
/// \param blockIdRow, blockIdCol The block indices in matrix
/// \param[in,out] matrix The matrix to add the sub-matrix into
/// \param initialize Option parameter, default=true. If true, the matrix form is assumed to be undefined
/// and is initialized when necessary. If false, the matrix form is assumed to be previously defined.
/// \note This is a specialization of addSubMatrix for sparse matrices.
template <typename DerivedSub, typename T, int Opt, typename Index>
void addSubMatrix(const DerivedSub& subMatrix, Index blockIdRow, Index blockIdCol,
				  Eigen::SparseMatrix<T, Opt, Index>* matrix, bool initialize = true)
{
	if (initialize)
	{
		blockOperation(subMatrix, static_cast<Index>(subMatrix.rows() * blockIdRow),
					   static_cast<Index>(subMatrix.cols() * blockIdCol), matrix,
					   &Operation<T, Opt, Index, DerivedSub>::add);
	}
	else
	{
		blockWithSearch(subMatrix, static_cast<Index>(subMatrix.rows() * blockIdRow),
						static_cast<Index>(subMatrix.cols() * blockIdCol),
						static_cast<Index>(subMatrix.rows()), static_cast<Index>(subMatrix.cols()), matrix,
						&Operation<T, Opt, Index, DerivedSub>::add);
	}
}

/// Helper method to zero a row of a matrix specialized for Sparse Matrices
/// \param row The row to set to zero
/// \param[in,out] matrix The matrix to set the zero row on.
template <typename T, int Opt, typename Index>
void zeroRow(Index row, Eigen::SparseMatrix<T, Opt, Index>* matrix)
{
	for (Index column = 0; column < matrix->cols(); ++column)
	{
		if (matrix->coeff(row, column))
		{
			matrix->coeffRef(row, column) = 0;
		}
	}
}

/// Helper method to zero a column of a matrix specialized for Sparse Matrices
/// \param column The column to set to zero
/// \param[in,out] matrix The matrix to set the zero column on.
template <typename T, int Opt, typename Index>
inline void zeroColumn(Index column, Eigen::SparseMatrix<T, Opt, Index>* matrix)
{
	for (Index row = 0; row < matrix->rows(); ++row)
	{
		if (matrix->coeff(row, column))
		{
			matrix->coeffRef(row, column) = 0;
		}
	}
}

/// Helper method to zero all entries of a matrix specialized for Sparse Matrices. This
/// allows the preservation of the the matrix form while still allowing the reset of
/// the matrix entries to zero.
/// \param[in,out] matrix The matrix to set to zero
template <typename T, int Opt, typename Index>
inline void clearMatrix(Eigen::SparseMatrix<T, Opt, Index>* matrix)
{
	SURGSIM_ASSERT(matrix->isCompressed()) << "Invalid matrix. Matrix must be in compressed form.";
	T* ptr = matrix->valuePtr();
	for (Index value = 0; value < matrix->nonZeros(); ++value)
	{
		*ptr = 0;
		++ptr;
	}
}

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_SPARSEMATRIX_H

// This file is a part of the OpenSurgSim project.
// Copyright 2012-2015, SimQuest Solutions Inc.
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
#include "SurgSim/Math/Matrix.h"

namespace SurgSim
{
namespace Math
{
/// A sparse matrix
typedef Eigen::SparseMatrix<double> SparseMatrix;

/// Helper class to run operation a column/row of a matrix to a chunk of memory where the size is dynamically defined
/// \tparam DerivedSub The class of the matrix from which the data are copied
/// \tparam SparseType The type of the SparseVector/SparseMatrix containing the chunk of memory
/// \tparam StorageOrder The storage option of the SparseType
template < class DerivedSub, class SparseType,
		   int StorageOrder =
		   ((SparseType::Flags & Eigen::RowMajorBit) == Eigen::RowMajorBit) ? Eigen::RowMajor : Eigen::ColMajor >
class Operation
{
public :
	typedef typename SparseType::Scalar T;
	typedef typename SparseType::StorageIndex StorageIndex;

	/// Do the assignment of a row/column of a matrix to a chunk of memory
	/// \param ptr The chunk of memory
	/// \param start Where the assignment starts in the chunk of memory
	/// \param n, m The size of the block (n rows, m columns)
	/// \param subMatrix The matrix from which the row/column is copied
	/// \param colRowId The column or row id depending on the template parameter Opt
	void assign(T* ptr, StorageIndex start, Eigen::Index n, Eigen::Index m,
		const Eigen::Ref<const DerivedSub>& subMatrix, Eigen::Index colRowId) {}

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
	void add(T* ptr, StorageIndex start, Eigen::Index n, Eigen::Index m,
		const Eigen::Ref<const DerivedSub>& subMatrix, Eigen::Index colRowId) {}

	/// Do the addition of a single matrix element (operator +=)
	/// \param ptr The matrix element to be increased
	/// \param value The value to add
	void add(T* ptr, const T& value) {}
};

/// Specialization for column major storage
template <class DerivedSub, class SparseType>
class Operation<DerivedSub, SparseType, Eigen::ColMajor>
{
public:
	typedef typename SparseType::Scalar T;
	typedef typename SparseType::StorageIndex StorageIndex;

	void assign(T* ptr, StorageIndex start, Eigen::Index n, Eigen::Index m,
		const Eigen::Ref<const DerivedSub>& subMatrix, Eigen::Index colRowId)
	{
		typedef Eigen::Matrix < T, Eigen::Dynamic, 1, Eigen::DontAlign | Eigen::ColMajor > ColVector;

		// ptr[start] is the 1st element in the column
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the operation
		Eigen::Map<ColVector>(&ptr[start], n).operator = (subMatrix.col(colRowId).segment(0, n));
	}

	void assign(T* ptr, const T& value)
	{
		*ptr = value;
	}

	void add(T* ptr, StorageIndex start, Eigen::Index n, Eigen::Index m,
		const Eigen::Ref<const DerivedSub>& subMatrix, Eigen::Index colRowId)
	{
		typedef Eigen::Matrix < T, Eigen::Dynamic, 1, Eigen::DontAlign | Eigen::ColMajor > ColVector;

		// ptr[start] is the 1st element in the column
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the operation
		Eigen::Map<ColVector>(&ptr[start], n).operator += (subMatrix.col(colRowId).segment(0, n));
	}

	void add(T* ptr, const T& value)
	{
		*ptr += value;
	}
};

/// Specialization for row major storage
template <class DerivedSub, class SparseType>
class Operation<DerivedSub, SparseType, Eigen::RowMajor>
{
public:
	typedef typename SparseType::Scalar T;
	typedef typename SparseType::StorageIndex StorageIndex;

	void assign(T* ptr, StorageIndex start, Eigen::Index n, Eigen::Index m,
		const Eigen::Ref<const DerivedSub>& subMatrix, Eigen::Index colRowId)
	{
		typedef Eigen::Matrix < T, 1, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor > RowVector;

		// ptr[start] is the 1st element in the row
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the operation
		Eigen::Map<RowVector>(&ptr[start], m).operator = (subMatrix.row(colRowId).segment(0, m));
	}

	void assign(T* ptr, const T& value)
	{
		*ptr = value;
	}

	void add(T* ptr, StorageIndex start, Eigen::Index n, Eigen::Index m,
		const Eigen::Ref<const DerivedSub>& subMatrix, Eigen::Index colRowId)
	{
		typedef Eigen::Matrix < T, 1, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor > RowVector;

		// ptr[start] is the 1st element in the row
		// The elements exists and are contiguous in memory, we use Eigen::Map functionality to optimize the operation
		Eigen::Map<RowVector>(&ptr[start], m).operator += (subMatrix.row(colRowId).segment(0, m));
	}

	void add(T* ptr, const T& value)
	{
		*ptr += value;
	}
};

/// Runs a given operation on a SparseMatrix block(i, j, n, m) from a (n x m) 'subMatrix' with searching for the
/// block 1st element. <br>
/// It supposes: <br>
/// + that the SparseMatrix already contains all the elements within the block (no insertion necessary) <br>
/// + that both the SparseMatrix and the 'subMatrix' are using the same Scalar type, double. <br>
/// This function will not change anything to the structure of the SparseMatrix, only change the values of the
/// corresponding coefficients.
/// \tparam Opt, StorageIndex Type parameters defining the output matrix type SparseMatrix<double, Opt, StorageIndex>
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
template <int Opt, typename StorageIndex>
void blockWithSearch(const Eigen::Ref<const Matrix>& subMatrix,
	Eigen::Index rowStart, Eigen::Index columnStart, Eigen::Index n, Eigen::Index m,
	Eigen::SparseMatrix<double, Opt, StorageIndex>* matrix,
	void (Operation<Matrix, Eigen::SparseMatrix<double, Opt, StorageIndex>>::*op)
	(double*, StorageIndex, Eigen::Index, Eigen::Index, const Eigen::Ref<const Matrix>&, Eigen::Index))
{
	static Operation<Matrix, Eigen::SparseMatrix<double, Opt, StorageIndex>> operation;
	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	SURGSIM_ASSERT(subMatrix.rows() >= n) << "subMatrix doesn't have enough rows";
	SURGSIM_ASSERT(subMatrix.cols() >= m) << "subMatrix doesn't have enough columns";

	SURGSIM_ASSERT(matrix->rows() >= rowStart + n) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->cols() >= columnStart + m) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->valuePtr() != nullptr) << "The matrix is not initialized correctly, null pointer to values";

	double* ptr = matrix->valuePtr();
	const StorageIndex* innerIndices = matrix->innerIndexPtr();
	const StorageIndex* outerIndices = matrix->outerIndexPtr();

	using Eigen::Index;
	const Index outerStart = (Opt == Eigen::ColMajor ? columnStart : rowStart);
	const Index innerStart = (Opt == Eigen::ColMajor ? rowStart : columnStart);
	const Index outerSize = (Opt == Eigen::ColMajor ? m : n);
	const Index innerSize = (Opt == Eigen::ColMajor ? n : m);

	for (Index outerLoop = 0; outerLoop < outerSize; ++outerLoop)
	{
		// outerIndices[outerStart + outerLoop] is the index in ptr and innerIndices of the first non-zero element in
		// the outer element (outerStart + outerLoop)
		const StorageIndex innerStartIdInCurrentOuter = outerIndices[outerStart + outerLoop];
		const StorageIndex innerStartIdInNextOuter = outerIndices[outerStart + outerLoop + 1];

		// Look for the index of innerStart in this outer (the column/row may contain elements before)
		StorageIndex innerFirstElement;
		if (static_cast<Index>(innerIndices[innerStartIdInCurrentOuter]) == innerStart)
		{
			innerFirstElement = innerStartIdInCurrentOuter;
		}
		else
		{
			innerFirstElement = static_cast<StorageIndex>(matrix->data().searchLowerIndex(
				static_cast<Index>(innerStartIdInCurrentOuter), static_cast<Index>(innerStartIdInNextOuter) - 1,
				innerStart));
		}

		// Make sure we actually found the 1st element of the block in this outer
		SURGSIM_ASSERT(static_cast<Index>(innerIndices[innerFirstElement]) == innerStart) <<
				"matrix is missing an element of the block (1st element on a row/column)";

		// Make sure that we are not going to write out of the range...
		// i.e. The column/row (starting at the beginning of the block) has at least innerSize elements
		SURGSIM_ASSERT(innerSize <= static_cast<Index>(innerStartIdInNextOuter - innerFirstElement)) <<
				"matrix is missing elements of the block (but not the 1st element on a row/column)";

		// Make sure that the last element corresponding to the block size has the expected index
		SURGSIM_ASSERT((innerStart + innerSize - 1) == \
					   static_cast<Index>(innerIndices[static_cast<Index>(innerFirstElement) + innerSize - 1])) <<
							   "The last element of the block does not have the expected index. " <<
							   "The matrix is missing elements of the block (but not the 1st element on a row/column)";

		(operation.*op)(ptr, innerFirstElement, n, m, subMatrix, outerLoop);
	}
}

/// Runs a given operation on a SparseMatrix block(i, j, n, m) from a (n x m) 'subMatrix' with searching for the
/// block 1st element. <br>
/// It supposes: <br>
/// + that both the SparseMatrix and the 'subMatrix' are using the same Scalar type, double. <br>
/// This function can be used to initialize the form of a sparse matrix as it will add entries when appropriate
/// entries do not exist.
/// \tparam Opt, StorageIndex Type parameters defining the output matrix type SparseMatrix<double, Opt, StorageIndex>
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
template <int Opt, typename StorageIndex>
void blockOperation(const Eigen::Ref<const Matrix>& subMatrix, Eigen::Index rowStart, Eigen::Index columnStart,
	Eigen::SparseMatrix<double, Opt, StorageIndex>* matrix,
	void (Operation<Matrix, Eigen::SparseMatrix<double, Opt, StorageIndex>>::*op)(double*, const double&))
{
	static Operation<Matrix, Eigen::SparseMatrix<double, Opt, StorageIndex>> operation;
	SURGSIM_ASSERT(nullptr != matrix) << "Invalid recipient matrix, nullptr found";

	using Eigen::Index;
	Index n = subMatrix.rows();
	Index m = subMatrix.cols();
	SURGSIM_ASSERT(matrix->rows() >= rowStart + n) << "The block is out of range in matrix";
	SURGSIM_ASSERT(matrix->cols() >= columnStart + m) << "The block is out of range in matrix";

	for (Index row = 0; row < n; ++row)
	{
		for (Index column = 0; column < m; ++column)
		{
			(operation.*op)(&matrix->coeffRef(rowStart + row, columnStart + column), subMatrix(row, column));
		}
	}
}

/// Add a dense sub-matrix into a matrix that may need the memory locations to be initialized.
/// \tparam Opt, StorageIndex Type parameters defining the output matrix type SparseMatrix<double, Opt, StorageIndex>
/// \param subMatrix The sub-matrix
/// \param blockIdRow, blockIdCol The block indices in matrix
/// \param[in,out] matrix The matrix to add the sub-matrix into
/// \note This is a specialization of addSubMatrix for sparse matrices.
/// \sa addSubMatrixNoInitialize
template <int Opt, typename StorageIndex>
void addSubMatrix(const Eigen::Ref<const Matrix>& subMatrix, Eigen::Index blockIdRow,
	Eigen::Index blockIdCol, Eigen::SparseMatrix<double, Opt, StorageIndex>* matrix)
{
	blockOperation(subMatrix, subMatrix.rows() * blockIdRow, subMatrix.cols() * blockIdCol, matrix,
		&Operation<Matrix, Eigen::SparseMatrix<double, Opt, StorageIndex>>::add);
}

/// Add a dense sub-matrix into a sparse matrix that does not need the memory locations to be initialized.
/// \tparam Opt, StorageIndex Type parameters defining the output matrix type SparseMatrix<double, Opt, StorageIndex>
/// \param subMatrix The sub-matrix
/// \param blockIdRow, blockIdCol The block indices in matrix
/// \param[in,out] matrix The matrix to add the sub-matrix into
/// \note This is a specialization of addSubMatrix for sparse matrices.
/// \sa addSubMatrix
template <int Opt, typename StorageIndex>
void addSubMatrixNoInitialize(const Eigen::Ref<const Matrix>& subMatrix, Eigen::Index blockIdRow,
	Eigen::Index blockIdCol, Eigen::SparseMatrix<double, Opt, StorageIndex>* matrix)
{
	blockWithSearch(subMatrix, subMatrix.rows() * blockIdRow, subMatrix.cols() * blockIdCol,
		subMatrix.rows(), subMatrix.cols(), matrix,
		&Operation<Matrix, Eigen::SparseMatrix<double, Opt, StorageIndex>>::add);
}

/// Assign a dense sub-matrix into a matrix that may need the memory locations to be initialized.
/// \tparam Opt, StorageIndex Type parameters defining the output matrix type SparseMatrix<double, Opt, Index>
/// \param subMatrix The sub-matrix
/// \param blockIdRow, blockIdCol The block indices in matrix
/// \param[in,out] matrix The matrix to assign the sub-matrix into
/// \sa assignSubMatrixNoInitialize
template <int Opt, typename StorageIndex>
void assignSubMatrix(const Eigen::Ref<const Matrix>& subMatrix, Eigen::Index blockIdRow, Eigen::Index blockIdCol,
					 Eigen::SparseMatrix<double, Opt, StorageIndex>* matrix)
{
	blockOperation(subMatrix, subMatrix.rows() * blockIdRow, subMatrix.cols() * blockIdCol, matrix,
		&Operation<Matrix, Eigen::SparseMatrix<double, Opt, StorageIndex>>::assign);
}

/// Assign a dense sub-matrix into a sparse matrix that does not need the memory locations to be initialized.
/// \tparam Opt, StorageIndex Type parameters defining the output matrix type SparseMatrix<double, Opt, Index>
/// \param subMatrix The sub-matrix
/// \param blockIdRow, blockIdCol The block indices in matrix
/// \param[in,out] matrix The matrix to assign the sub-matrix into
/// \sa assignSubMatrix
template <int Opt, typename StorageIndex>
void assignSubMatrixNoInitialize(const Eigen::Ref<const Matrix>& subMatrix,
	Eigen::Index blockIdRow, Eigen::Index blockIdCol, Eigen::SparseMatrix<double, Opt, StorageIndex>* matrix)
{
	blockWithSearch(subMatrix, subMatrix.rows() * blockIdRow, subMatrix.cols() * blockIdCol,
		subMatrix.rows(), subMatrix.cols(), matrix,
		&Operation<Matrix, Eigen::SparseMatrix<double, Opt, StorageIndex>>::assign);
}

/// Helper method to zero a row of a matrix specialized for Sparse Matrices
/// \param row The row to set to zero
/// \param[in,out] matrix The matrix to set the zero row on.
template <typename T, int Opt, typename StorageIndex>
void zeroRow(Eigen::Index row, Eigen::SparseMatrix<T, Opt, StorageIndex>* matrix)
{
	for (Eigen::Index column = 0; column < matrix->cols(); ++column)
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
template <typename T, int Opt, typename StorageIndex>
inline void zeroColumn(Eigen::Index column, Eigen::SparseMatrix<T, Opt, StorageIndex>* matrix)
{
	for (Eigen::Index row = 0; row < matrix->rows(); ++row)
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
template <typename T, int Opt, typename StorageIndex>
inline void clearMatrix(Eigen::SparseMatrix<T, Opt, StorageIndex>* matrix)
{
	SURGSIM_ASSERT(matrix->isCompressed()) << "Invalid matrix. Matrix must be in compressed form.";
	T* ptr = matrix->valuePtr();
	for (Eigen::Index value = 0; value < matrix->nonZeros(); ++value)
	{
		*ptr = 0;
		++ptr;
	}
}

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_SPARSEMATRIX_H

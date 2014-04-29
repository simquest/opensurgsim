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

#ifndef SURGSIM_MATH_LINEARSOLVEANDINVERSE_INL_H
#define SURGSIM_MATH_LINEARSOLVEANDINVERSE_INL_H

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{

namespace Math
{

template <int BlockSize>
const Eigen::Block<const Matrix, BlockSize, BlockSize>
	LinearSolveAndInverseTriDiagonalBlockMatrix<BlockSize>::minusAi(const SurgSim::Math::Matrix& A, size_t i) const
{
	return A.block<BlockSize, BlockSize>(BlockSize * i, BlockSize * (i - 1));
}

template <int BlockSize>
const Eigen::Block<const Matrix, BlockSize, BlockSize>
	LinearSolveAndInverseTriDiagonalBlockMatrix<BlockSize>::Bi(const SurgSim::Math::Matrix& A, size_t i) const
{
	return A.block<BlockSize, BlockSize>(BlockSize * i, BlockSize * i);
}

template <int BlockSize>
const Eigen::Block<const Matrix, BlockSize, BlockSize>
	LinearSolveAndInverseTriDiagonalBlockMatrix<BlockSize>::minusCi(const SurgSim::Math::Matrix& A, size_t i) const
{
	return A.block<BlockSize, BlockSize>(BlockSize * i, BlockSize * (i + 1));
}

template <int BlockSize>
void LinearSolveAndInverseTriDiagonalBlockMatrix<BlockSize>::inverseTriDiagonalBlock(const SurgSim::Math::Matrix& A,
																			   SurgSim::Math::Matrix* inv,
																			   bool isSymmetric)
{
	SURGSIM_ASSERT(inv != nullptr) << "Null inverse matrix pointer";

	SURGSIM_ASSERT(A.cols() == A.rows()) <<
		"Cannot inverse a non square tri-diagonal block matrix ("<< A.rows() <<" x "<< A.cols() <<")";

	const size_t size = A.rows();
	const size_t numBlocks = size / BlockSize;

	SURGSIM_ASSERT(numBlocks * BlockSize == size) <<
		"Bad tri-diagonal block matrix structure, size = " << size << " block size = " << BlockSize <<
		" and the number of blocks are " << numBlocks;

	// If the matrix size is less or equal to 4 (Eigen inverse use co-factor for those), or the matrix is
	// composed of an unique block, simply call the normal Eigen inverse method.
	if (size <= 4 || numBlocks == static_cast<size_t>(1))
	{
		*inv = A.inverse();
		return;
	}

	if (inv->rows() != size || inv->cols() != size)
	{
		inv->resize(size, size);
	}

	m_Bi_AiDiminus1_inv.resize(numBlocks);
	m_Di.resize(numBlocks - 1);
	m_Ei.resize(numBlocks);	// Should be of size m_numBlocks - 1 too, but index 0 is undefined and we
							// decided to not change the indexing to not introduce any confusion

	// Bi_AiDiminus1_inv[0] = (B0)^-1
	// D                [0] = (B0)^-1.C0
	m_Bi_AiDiminus1_inv[0] = Bi(A, 0).inverse();
	m_Di[0] = m_Bi_AiDiminus1_inv[0] * (-minusCi(A, 0));
	// Bi_AiDiminus1_inv[i] = (Bi - Ai.D[i-1])^-1
	// Di               [i] = (Bi - Ai.D[i-1])^-1 . Ci
	for(size_t i = 1; i < numBlocks - 1; i++)
	{
		m_Bi_AiDiminus1_inv[i] = (Bi(A, i) - (-minusAi(A, i)) * m_Di[i - 1]).inverse();
		m_Di[i] = m_Bi_AiDiminus1_inv[i] * (-minusCi(A, i));
	}
	// Bi_AiDiminus1_inv[nbBlock-1] = (B(nbBlock-1) - A(nbBlock-1).D(nbBlock-2))^-1
	// D                [nbBlock-1] = UNDEFINED because C(nbBlock-1) does not exist
	m_Bi_AiDiminus1_inv[numBlocks - 1] =
		(Bi(A, numBlocks - 1) - (-minusAi(A, numBlocks - 1)) * m_Di[numBlocks - 2]).inverse();

	// E[nbBlock-1] = (B(nbBlock-1))^-1 . A(nbBlock-1)
	// Ei           = (Bi - Ci.E(i+1))^-1 . Ai
	// E0           = UNDEFINED because A0 does not exist
	m_Ei[numBlocks - 1] = Bi(A, numBlocks - 1).inverse() * (-minusAi(A, numBlocks - 1));
	for(int i = static_cast<int>(numBlocks) - 2; i > 0; --i)
	{
		m_Ei[i] = (Bi(A, i) - (-minusCi(A, i)) * m_Ei[i + 1]).inverse() * (-minusAi(A, i));
	}

	// Inverse diagonal blocks:
	// inv(i,i) = (I - Di.E(i+1))^-1.Bi_AiDiminus1_inv[i]
	for(size_t i = 0; i < numBlocks - 1; ++i)
	{
		inv->block<BlockSize, BlockSize>(BlockSize * i, BlockSize * i) =
			(Block::Identity() - m_Di[i] * m_Ei[i + 1]).inverse() * m_Bi_AiDiminus1_inv[i];
	}
	// inv(nbBlock-1,nbBlock-1) = Bi_AiDiminus1_inv[nbBlock-1]
	inv->block<BlockSize, BlockSize>(BlockSize * (numBlocks - 1), BlockSize * (numBlocks - 1)) =
		m_Bi_AiDiminus1_inv[numBlocks - 1];

	// Inverse off-diagonal blocks:
	// inv(i,j) = Di.inv(i+1,j) for i<j
	// inv(i,j) = Ei.inv(i-1,j) for i>j
	if (isSymmetric)
	{
		for(int j = 0; j < static_cast<int>(numBlocks); ++j)
		{
			for(int i = j - 1; i >= 0; --i)
			{
				inv->block<BlockSize, BlockSize>(BlockSize * i, BlockSize * j) =
					m_Di[i] * inv->block<BlockSize, BlockSize>(BlockSize * (i + 1), BlockSize * j);
				inv->block<BlockSize, BlockSize>(BlockSize * j, BlockSize * i) =
					inv->block<BlockSize, BlockSize>(BlockSize * i, BlockSize * j).transpose();
			}
		}
	}
	else
	{
		for(int j = 0; j < static_cast<int>(numBlocks); ++j)
		{
			for(int i = j - 1; i >= 0; --i)
			{
				inv->block<BlockSize, BlockSize>(BlockSize * i, BlockSize * j) =
					m_Di[i] * inv->block<BlockSize, BlockSize>(BlockSize * (i + 1), BlockSize * j);
			}
			for(int i = j + 1; i < static_cast<int>(numBlocks); ++i)
			{
				inv->block<BlockSize, BlockSize>(BlockSize * i, BlockSize * j) =
					m_Ei[i] * inv->block<BlockSize, BlockSize>(BlockSize * (i - 1), BlockSize * j);
			}
		}
	}
}

template <int BlockSize>
void LinearSolveAndInverseTriDiagonalBlockMatrix<BlockSize>::operator ()(const Matrix& A, const Vector& b, Vector* x,
																   Matrix* Ainv)
{
	SURGSIM_ASSERT(A.cols() == A.rows()) << "Cannot inverse a non square matrix";

	if (Ainv != nullptr)
	{
		inverseTriDiagonalBlock(A, Ainv);
		(*x) = (*Ainv) * b;
	}
	else if (x != nullptr)
	{
		if (m_inverse.rows() != A.rows() || m_inverse.cols() != A.cols())
		{
			m_inverse.resize(A.rows(), A.cols());
		}
		inverseTriDiagonalBlock(A, &m_inverse);
		(*x) = m_inverse * b;
	}
}

};  // namespace Math

};  // namespace SurgSim

#endif // SURGSIM_MATH_LINEARSOLVEANDINVERSE_INL_H

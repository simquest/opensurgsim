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

/// \file
/// Tests for the LinearSolveAndInverse.cpp functions.

#include <gtest/gtest.h>

#include "SurgSim/Math/LinearSolveAndInverse.h"

namespace SurgSim
{

namespace Math
{

class LinearSolveAndInverseTests : public ::testing::Test
{
private:
	size_t size;

	void initializeVector(Vector* v)
	{
		v->resize(size);
		for (size_t row = 0; row < size; row++)
		{
			(*v)(row) = fmod(-4.1 * row * row + 3.46, 5.0);
		}
	}

	void initializeDenseMatrix(Matrix* m)
	{
		m->resize(size, size);
		for (size_t row = 0; row < size; row++)
		{
			for (size_t col = 0; col < size; col++)
			{
				(*m)(row, col) = fmod((10.3 * cos(static_cast<double>(row * col)) + 3.24), 10.0);
			}
			}
	}

	void initializeDiagonalMatrix(Matrix* m)
	{
		m->resize(size, size);
		m->setZero();
		for (size_t row = 0; row < size; row++)
		{
			(*m)(row, row) = fmod((10.3 * cos(static_cast<double>(row * row)) + 3.24), 10.0);
		}
	}

	template <size_t BlockSize>
	void initializeTriDiagonalBlockMatrix(Matrix* m, bool isSymmetric)
	{
		size_t numBlocks = size / BlockSize;

		m->resize(size, size);
		m->setZero();

		for (size_t rowBlockId = 0; rowBlockId < numBlocks; rowBlockId++)
		{
			for (int colBlockId = static_cast<int>(rowBlockId) - 1;
				colBlockId <= static_cast<int>(rowBlockId) + 1;
				colBlockId++)
			{
				if (colBlockId < 0 || colBlockId >= static_cast<int>(numBlocks))
				{
					continue;
				}

				for (size_t rowInBlockId = 0; rowInBlockId < BlockSize; ++rowInBlockId)
				{
					for (size_t colInBlockId = 0; colInBlockId < BlockSize; ++colInBlockId)
					{
						size_t row = rowBlockId * BlockSize + rowInBlockId;
						size_t col = colBlockId * BlockSize + colInBlockId;
						(*m)(row, col) = fmod((10.3 * cos(static_cast<double>(row * col)) + 3.24), 10.0);
					}
				}
			}
		}

		if (isSymmetric)
		{
			// Force symmetry (lower triangular is copied from the upper triangular)
			for (size_t row = 0; row < size; ++row)
			{
				for (size_t col = row + 1; col < size; ++col)
				{
					(*m)(col, row) = (*m)(row, col);
				}
			}
		}
	}

	void setupTest()
	{
		initializeVector(&b);
		expectedInverse = matrix.inverse();
		expectedX = expectedInverse * b;
	}

public:

	void setupDenseMatrixTest()
	{
		size = 18;
		initializeDenseMatrix(&matrix);
		setupTest();
	}

	void setupDiagonalMatrixTest()
	{
		size = 18;
		initializeDiagonalMatrix(&matrix);
		setupTest();
	}

	template <size_t BlockSize>
	void setupTriDiagonalBlockMatrixTest(bool isSymmetric = false)
	{
		size = BlockSize * 6;
		initializeTriDiagonalBlockMatrix<BlockSize>(&matrix, isSymmetric);
		setupTest();
	}

	Matrix matrix;
	Matrix inverseMatrix, expectedInverse;
	Vector b;
	Vector x, expectedX;
};

TEST_F(LinearSolveAndInverseTests, DenseMatrixTests)
{
	setupDenseMatrixTest();

	LinearSolveAndInverseDenseMatrix solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseTests, DiagonalMatrixTests)
{
	setupDiagonalMatrixTest();

	LinearSolveAndInverseDiagonalMatrix solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseTests, TriDiagonalBlockMatrixBlockSize2Tests)
{
	setupTriDiagonalBlockMatrixTest<2>();

	LinearSolveAndInverseTriDiagonalBlockMatrix<2> solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseTests, TriDiagonalBlockMatrixBlockSize3Tests)
{
	setupTriDiagonalBlockMatrixTest<3>();

	LinearSolveAndInverseTriDiagonalBlockMatrix<3> solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseTests, TriDiagonalBlockMatrixBlockSize4Tests)
{
	setupTriDiagonalBlockMatrixTest<4>();

	LinearSolveAndInverseTriDiagonalBlockMatrix<4> solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseTests, TriDiagonalBlockMatrixBlockSize5Tests)
{
	setupTriDiagonalBlockMatrixTest<5>();

	LinearSolveAndInverseTriDiagonalBlockMatrix<5> solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseTests, TriDiagonalBlockMatrixBlockSize6Tests)
{
	setupTriDiagonalBlockMatrixTest<6>();

	LinearSolveAndInverseTriDiagonalBlockMatrix<6> solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseTests, SymmetricTriDiagonalBlockMatrixBlockSize2Tests)
{
	setupTriDiagonalBlockMatrixTest<2>(true);

	LinearSolveAndInverseSymmetricTriDiagonalBlockMatrix<2> solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseTests, SymmetricTriDiagonalBlockMatrixBlockSize3Tests)
{
	setupTriDiagonalBlockMatrixTest<3>(true);

	LinearSolveAndInverseSymmetricTriDiagonalBlockMatrix<3> solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseTests, SymmetricTriDiagonalBlockMatrixBlockSize4Tests)
{
	setupTriDiagonalBlockMatrixTest<4>(true);

	LinearSolveAndInverseSymmetricTriDiagonalBlockMatrix<4> solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseTests, SymmetricTriDiagonalBlockMatrixBlockSize5Tests)
{
	setupTriDiagonalBlockMatrixTest<5>(true);

	LinearSolveAndInverseSymmetricTriDiagonalBlockMatrix<5> solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseTests, SymmetricTriDiagonalBlockMatrixBlockSize6Tests)
{
	setupTriDiagonalBlockMatrixTest<6>(true);

	LinearSolveAndInverseSymmetricTriDiagonalBlockMatrix<6> solveAndInverse;
	solveAndInverse.update(matrix);
	solveAndInverse.solve(b, &x);
	solveAndInverse.getInverse(&inverseMatrix);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));
};
}; // namespace Math

}; // namespace SurgSim

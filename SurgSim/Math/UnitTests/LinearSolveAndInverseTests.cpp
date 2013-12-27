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
///

#include <gtest/gtest.h>

#include "SurgSim/Math/LinearSolveAndInverse.h"

namespace SurgSim
{

namespace Math
{

namespace
{
	const int size = 10;
	SurgSim::Math::Matrix gMatrix(size, size);
	SurgSim::Math::Vector gVector(size);

	void initializeMatrixVector()
	{
		for (size_t row = 0; row < size; row++)
		{
			for (size_t col = 0; col < size; col++)
			{
				gMatrix(row, col) = 1.47 * sqrt((static_cast<double>(row + 1))) + 8.3 * row * col - 0.24 * col + 13.24;
			}
			gVector(row) = -4.1 * row * row + 3.46;
		}
	}
};


class LinearSolveAndInverseMatrixTests : public ::testing::Test
{
public:
	static void SetUpTestCase()
	{
		initializeMatrixVector();
	}

	LinearSolveAndInverseMatrixTests()
	{
		resize(&matrix, size, size, false);
		matrix = gMatrix;
		expectedInverse = matrix.inverse();
		resize(&b, size);
		b = gVector;
		resize(&x, size);
		expectedX = expectedInverse * b;
	}
	Matrix matrix;
	Matrix inverse, expectedInverse;
	Vector b;
	Vector x, expectedX;
};

class LinearSolveAndInverseDiagonalMatrixTests : public ::testing::Test
{
public:
	static void SetUpTestCase()
	{
		initializeMatrixVector();
	}

	LinearSolveAndInverseDiagonalMatrixTests()
	{
		resize(&matrix, size, size, false);
		matrix.diagonal() = gMatrix.diagonal();
		denseMatrix = matrix;
		expectedInverse = denseMatrix.inverse();
		resize(&b, size);
		b = gVector;
		resize(&x, size);
		expectedX = expectedInverse * b;
	}
	DiagonalMatrix matrix;
	Matrix denseMatrix;
	Matrix expectedInverse;
	Matrix inverse;
	Vector b;
	Vector x, expectedX;
};

class LinearSolveAndInverseSparseMatrixTests : public ::testing::Test
{
public:
	static void SetUpTestCase()
	{
		initializeMatrixVector();
	}

	LinearSolveAndInverseSparseMatrixTests() : matrix(size, size)
	{
		std::vector<Eigen::Triplet<double>> coefficients; // list of non-zeros coefficients
		coefficients.reserve(size + size-1+ size-1); // Tri diagonal matrix structure
		for(int rowId = 0; rowId < size; rowId++)
		{
			// Tri diagonal matrix structure
			if (rowId > 0)
			{
				coefficients.push_back(Eigen::Triplet<double>(rowId, rowId - 1, 3.2 - 3.24*rowId));
			}
			coefficients.push_back(Eigen::Triplet<double>(rowId, rowId, 3.4 + rowId));
			if (rowId < size - 1)
			{
				coefficients.push_back(Eigen::Triplet<double>(rowId, rowId + 1, 8.12 + 1.9*rowId));
			}
		}
		matrix.setFromTriplets(coefficients.begin(), coefficients.end());
		denseMatrix = matrix;
		expectedInverse = denseMatrix.inverse();
		resize(&b, size);
		b = gVector;
		resize(&x, size);
		expectedX = expectedInverse * b;
	}
	Eigen::SparseMatrix<double,Eigen::ColMajor> matrix;
	Matrix denseMatrix;
	Matrix expectedInverse;
	Matrix inverse;
	Vector b;
	Vector x, expectedX;
};

TEST_F(LinearSolveAndInverseDiagonalMatrixTests, solve)
{
	SolveAndInverse<DiagonalMatrix> solveAndInverse;
	solveAndInverse(matrix, b, &x, &inverse);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverse.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseMatrixTests, solve)
{
	SolveAndInverse<Matrix> solveAndInverse;
	solveAndInverse(matrix, b, &x, &inverse);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverse.isApprox(expectedInverse));
};

TEST_F(LinearSolveAndInverseSparseMatrixTests, solve)
{
	SolveAndInverse<Eigen::SparseMatrix<double,Eigen::ColMajor>> solveAndInverse;
	solveAndInverse(matrix, b, &x, &inverse);

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverse.isApprox(expectedInverse));
};

}; // namespace Math

}; // namespace SurgSim

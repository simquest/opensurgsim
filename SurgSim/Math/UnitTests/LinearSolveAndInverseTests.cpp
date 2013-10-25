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

#include <SurgSim/Math/LinearSolveAndInverse.h>

namespace SurgSim
{

namespace Math
{

namespace
{
	const int size = 10;
};

template <class T>
class LinearSolveAndInverseTests : public ::testing::Test
{
public:
	LinearSolveAndInverseTests(){}
};

template <>
class LinearSolveAndInverseTests<Matrix> : public ::testing::Test
{
public:
	LinearSolveAndInverseTests()
	{
		resize(&matrix, size, size, false);
		matrix.setRandom();
		expectedInverse = matrix.inverse();
		resize(&b, size);
		b.setRandom();
		resize(&x, size);
		expectedX = expectedInverse * b;
	}
	Matrix matrix;
	Matrix inverse, expectedInverse;
	Vector b;
	Vector x, expectedX;
};
typedef LinearSolveAndInverseTests<Matrix> LinearSolveAndInverseMatrixTests;

template <>
class LinearSolveAndInverseTests<DiagonalMatrix> : public ::testing::Test
{
public:
	LinearSolveAndInverseTests()
	{
		resize(&matrix, size, size, false);
		matrix.diagonal().setRandom();
		denseMatrix = matrix;
		expectedInverse = denseMatrix.inverse();
		resize(&b, size);
		b.setRandom();
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
typedef LinearSolveAndInverseTests<DiagonalMatrix> LinearSolveAndInverseDiagonalMatrixTests;

template <>
class LinearSolveAndInverseTests<Eigen::SparseMatrix<double,Eigen::ColMajor>> : public ::testing::Test
{
public:
	LinearSolveAndInverseTests() : matrix(size, size)
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
		b.setRandom();
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
typedef LinearSolveAndInverseTests<Eigen::SparseMatrix<double,Eigen::ColMajor>> LinearSolveAndInverseSparseMatrixTests;

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

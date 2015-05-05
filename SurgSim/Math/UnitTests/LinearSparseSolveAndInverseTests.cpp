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
/// Tests for the LinearSparseSolveAndInverse.cpp functions.

#include <gtest/gtest.h>

#include "SurgSim/Math/LinearSparseSolveAndInverse.h"

namespace SurgSim
{

namespace Math
{

class LinearSparseSolveAndInverseTests : public ::testing::Test
{
public:

	void setupSparseMatrixTest()
	{
		inverseMatrix.resize(size, size);
		initializeSparseMatrix(&matrix);
	}

	SparseMatrix matrix;
	Matrix denseMatrix, inverseMatrix, expectedInverse;
	Vector b;
	Vector x, expectedX;

protected:
	void SetUp() override
	{
		size = 18;
		initializeVector(&b);
		initializeDenseMatrix(&denseMatrix);
		expectedInverse = denseMatrix.inverse();
		expectedX = expectedInverse * b;
	}

private:
	size_t size;

	void initializeVector(Vector* v)
	{
		v->resize(size);
		for (size_t row = 0; row < size; row++)
		{
			(*v)(row) = std::fmod(-4.1 * row * row + 3.46, 5.0);
		}
	}

	void initializeSparseMatrix(SparseMatrix* m)
	{
		m->resize(static_cast<SparseMatrix::Index>(size), static_cast<SparseMatrix::Index>(size));
		for (SparseMatrix::Index row = 0; row < size; row++)
		{
			for (SparseMatrix::Index col = 0; col < size; col++)
			{
				(*m).insert(row, col) =
					std::fmod((10.3 * std::cos(static_cast<double>(row * col)) + 3.24), 10.0);
			}
		}
		m->makeCompressed();
	}

	void initializeDenseMatrix(Matrix* m)
	{
		m->resize(size, size);
		for (size_t row = 0; row < size; row++)
		{
			for (size_t col = 0; col < size; col++)
			{
				(*m)(row, col) = std::fmod((10.3 * std::cos(static_cast<double>(row * col)) + 3.24), 10.0);
			}
		}
	}
};

TEST_F(LinearSparseSolveAndInverseTests, SparseLUInitializationTests)
{
	SparseMatrix nonSquare(9, 18);
	nonSquare.setZero();

	LinearSparseSolveAndInverseLU solveAndInverse;
	EXPECT_THROW(solveAndInverse.setMatrix(nonSquare), SurgSim::Framework::AssertionFailure);
};

TEST_F(LinearSparseSolveAndInverseTests, SparseLUMatrixComponentsTest)
{
	setupSparseMatrixTest();

	LinearSparseSolveAndInverseLU solveAndInverse;
	solveAndInverse.setMatrix(matrix);
	x = solveAndInverse.solve(b);
	inverseMatrix = solveAndInverse.getInverse();

	EXPECT_TRUE(x.isApprox(expectedX));
	EXPECT_TRUE(inverseMatrix.isApprox(expectedInverse));

	inverseMatrix = solveAndInverse.solve(denseMatrix);
	EXPECT_TRUE(inverseMatrix.isApprox(Matrix::Identity(18, 18)));
};

TEST_F(LinearSparseSolveAndInverseTests, SparseCGSetGetTests)
{
	LinearSparseSolveAndInverseCG solveAndInverse;

	EXPECT_NE(solveAndInverse.getMaxIterations(), 100);
	solveAndInverse.setMaxIterations(100);
	EXPECT_EQ(solveAndInverse.getMaxIterations(), 100);

	EXPECT_NE(solveAndInverse.getTolerance(), 1.0e-03);
	solveAndInverse.setTolerance(1.0e-03);
	EXPECT_EQ(solveAndInverse.getTolerance(), 1.0e-03);
};

TEST_F(LinearSparseSolveAndInverseTests, SparseCGInitializationTests)
{
	SparseMatrix nonSquare(9, 18);
	nonSquare.setZero();

	LinearSparseSolveAndInverseCG solveAndInverse;
	EXPECT_THROW(solveAndInverse.setMatrix(nonSquare), SurgSim::Framework::AssertionFailure);
};

TEST_F(LinearSparseSolveAndInverseTests, SparseCGMatrixComponentsTest)
{
	setupSparseMatrixTest();
	double lowPrecision = 1.0e-05;
	double highPrecision = 1.0e-10;

	LinearSparseSolveAndInverseCG solveAndInverse;
	solveAndInverse.setMatrix(matrix);
	x = solveAndInverse.solve(b);
	inverseMatrix = solveAndInverse.getInverse();

	EXPECT_TRUE(x.isApprox(expectedX, lowPrecision)) << std::endl << "x: " << x.transpose() << std::endl <<
			"Expected: " << expectedX.transpose() << std::endl;
	EXPECT_FALSE(x.isApprox(expectedX, highPrecision)) << std::endl << "x: " << x.transpose() << std::endl <<
			"Expected: " << expectedX.transpose() << std::endl;

	inverseMatrix = solveAndInverse.solve(denseMatrix);
	EXPECT_TRUE(inverseMatrix.isApprox(Matrix::Identity(18, 18), lowPrecision)) << std::endl << "Identity: " <<
			inverseMatrix << std::endl;
	EXPECT_FALSE(inverseMatrix.isApprox(Matrix::Identity(18, 18), highPrecision)) << std::endl << "Identity: " <<
			inverseMatrix << std::endl;

	solveAndInverse.setTolerance(highPrecision);
	solveAndInverse.setMaxIterations(20);
	x = solveAndInverse.solve(b);
	EXPECT_TRUE(x.isApprox(expectedX, lowPrecision)) << std::endl << "x: " << x.transpose() << std::endl <<
			"Expected: " << expectedX.transpose() << std::endl;
	EXPECT_FALSE(x.isApprox(expectedX, highPrecision)) << std::endl << "x: " << x.transpose() << std::endl <<
			"Expected: " << expectedX.transpose() << std::endl;

	solveAndInverse.setMaxIterations(50);
	x = solveAndInverse.solve(b);
	EXPECT_TRUE(x.isApprox(expectedX, highPrecision)) << std::endl << "x: " << x.transpose() << std::endl <<
			"Expected: " << expectedX.transpose() << std::endl;

};

}; // namespace Math

}; // namespace SurgSim

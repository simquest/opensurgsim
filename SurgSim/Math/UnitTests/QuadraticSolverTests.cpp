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

/// \file QuadraticSolverTests.cpp
/// Tests for the quadratic solver function.

#include <gtest/gtest.h>

#include "SurgSim/Math/QuadraticSolver.h"

namespace SurgSim
{

namespace Math
{

TEST(QuadraticSolverTests, DegenerateCases)
{
	double roots[2];
	int numberOfRoots;

	{
		SCOPED_TRACE("Not quadratic, not linear, no solution: 0.x^2 + 0.x + 1 = 0");
		EXPECT_NO_THROW(numberOfRoots = findRoots(0.0, 0.0, 1.0, roots));
		EXPECT_EQ(0, numberOfRoots);
	}

	{
		SCOPED_TRACE("Not quadratic, not linear, all solutions: 0.x^2 + 0.x + 0 = 0");
		EXPECT_NO_THROW(numberOfRoots = findRoots(0.0, 0.0, 0.0, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_DOUBLE_EQ(0.0, roots[0]);
	}

	{
		SCOPED_TRACE("Not quadratic, linear: 0.x^2 + 3.0.x + 2.0 = 0");
		EXPECT_NO_THROW(numberOfRoots = findRoots(0.0, 3.0, 2.0, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_DOUBLE_EQ(-2.0 / 3.0, roots[0]);
	}
};

TEST(QuadraticSolverTests, NullDeterminantCases)
{
	double roots[2];
	int numberOfRoots;

	{
		SCOPED_TRACE("x^2 = 0 => determinant = 0*0 - 4*1*0 = 0");
		EXPECT_NO_THROW(numberOfRoots = findRoots(1.0, 0.0, 0.0, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_DOUBLE_EQ(0.0, roots[0]);
	}

	{
		SCOPED_TRACE("3x^2 + 6x + 3 = 0 => determinant = 6*6 - 4*3*3 = 0");
		EXPECT_NO_THROW(numberOfRoots = findRoots(3.0, 6.0, 3.0, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_DOUBLE_EQ(-1.0, roots[0]);
	}

	{
		SCOPED_TRACE("-3.254x^2 + sqrt(4 * -3.254 * -0.2345)x - 0.2345 = 0 => determinant = 0 by construction");
		double a = -3.254;
		double c = -0.2345;
		double b = sqrt(4.0 * a * c);
		EXPECT_NO_THROW(numberOfRoots = findRoots(a, b, c, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_DOUBLE_EQ(-b / (2.0 * a), roots[0]);
	}
};

TEST(QuadraticSolverTests, NegativeDeterminantCases)
{
	double roots[2];
	int numberOfRoots;

	{
		SCOPED_TRACE("x^2 + x + 1 = 0 => determinant = 1 - 4 = -3");
		EXPECT_NO_THROW(numberOfRoots = findRoots(1.0, 1.0, 1.0, roots));
		EXPECT_EQ(0, numberOfRoots);
	}

	{
		SCOPED_TRACE("-3.254x^2 + (sqrt(4 * -3.254 * -0.2345) - 1)x - 0.2345 = 0 => determinant < 0 by construction");
		double a = -3.254;
		double c = -0.2345;
		double b = sqrt(4.0 * a * c) - 1;
		EXPECT_NO_THROW(numberOfRoots = findRoots(a, b, c, roots));
		EXPECT_EQ(0, numberOfRoots);
	}
};

TEST(QuadraticSolverTests, PositiveDeterminantCases)
{
	double roots[2];
	int numberOfRoots;

	{
		SCOPED_TRACE("2x^2 + x - 1 = 0 => determinant = 1 + 8 = 9");
		EXPECT_NO_THROW(numberOfRoots = findRoots(2.0, 1.0, -1.0, roots));
		EXPECT_EQ(2, numberOfRoots);
		EXPECT_DOUBLE_EQ(-1.0, roots[0]); // The roots are ordered from lower to higher
		EXPECT_DOUBLE_EQ(0.5, roots[1]);
	}

	{
		SCOPED_TRACE("-3.254x^2 + (sqrt(4 * -3.254 * -0.2345) + 1)x - 0.2345 = 0 => determinant > 0 by construction");
		double a = -3.254;
		double c = -0.2345;
		double b = sqrt(4.0 * a * c) + 1;
		EXPECT_NO_THROW(numberOfRoots = findRoots(a, b, c, roots));
		EXPECT_EQ(2, numberOfRoots);
		double delta = b * b - 4.0 * a * c;
		EXPECT_DOUBLE_EQ((-b - std::sqrt(delta)) / (2.0 * a), roots[0]);
		EXPECT_DOUBLE_EQ((-b + std::sqrt(delta)) / (2.0 * a), roots[1]);
	}
};

}; // namespace Math
}; // namespace SurgSim

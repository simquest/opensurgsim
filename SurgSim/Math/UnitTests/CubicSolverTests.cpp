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

/// \file CubicSolverTests.cpp
/// Tests for the cubic solver function.

#include <gtest/gtest.h>

#include "SurgSim/Math/CubicSolver.h"

namespace SurgSim
{

namespace Math
{

TEST(CubicSolverTests, DegenerateCases)
{
	double roots[3];
	int numberOfRoots;

	{
		SCOPED_TRACE("0.x^3 + x^2 + x + 1 = 0 (no solution on R: none on [0..1])");
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(0.0, 1.0, 1.0, 1.0, roots));
		EXPECT_EQ(0, numberOfRoots);
	}

	{
		SCOPED_TRACE("0.x^3 + 1x^2 + 4x - 12 = 0 (2 solutions on R: -6 and 2, none on [0..1])");
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(0.0, 1.0, 4.0, -12.0, roots));
		EXPECT_EQ(0, numberOfRoots);
	}

	{
		SCOPED_TRACE("0.x^3 + x^2 + 0x + 0 = 0 (1 solution on R: 0, 1 on [0..1])");
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(0.0, 1.0, 0.0, 0.0, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_DOUBLE_EQ(0.0, roots[0]);
	}

	{
		SCOPED_TRACE("0.x^3 + x^2 + x - 2 = 0 (2 solutions on R: -2 and 1, 1 on [0..1])");
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(0.0, 1.0, 1.0, -2.0, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_DOUBLE_EQ(1.0, roots[0]);
	}

	{
		SCOPED_TRACE("0.x^3 + 2x^2 + x - 1 = 0 (2 solutions on R: -1 and 0.5, 1 on [0..1])");
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(0.0, 2.0, 1.0, -1.0, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_DOUBLE_EQ(0.5, roots[0]);
	}

};

TEST(CubicSolverTests, dNullCase)
{
	double roots[3];
	int numberOfRoots;

	{
		SCOPED_TRACE("x^3 + x^2 + x + 0 = 0");
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(1.0, 1.0, 1.0, 0.0, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_DOUBLE_EQ(0.0, roots[0]);
	}
}

TEST(CubicSolverTests, DerivativeNullDeterminantCases)
{
	double roots[3];
	int numberOfRoots;

	{
		SCOPED_TRACE("P(x) = -x^3 + 3x^2 - 3x + 1 = 0 => P'(x) = -3x^2 + 6x - 3 => discriminant = 36 - 4*(3)*(3) = 0");
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(-1.0, 3.0, -3.0, 1.0, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_DOUBLE_EQ(1.0, roots[0]);
	}

	{
		SCOPED_TRACE("P(x) = -x^3 + 3x^2 - 3x - 2 = 0 => P'(x) = -3x^2 + 6x - 3 => discriminant = 36 - 4*(3)*(3) = 0");
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(-1.0, 3.0, -3.0, -2.0, roots));
		EXPECT_EQ(0, numberOfRoots);
	}
};

TEST(CubicSolverTests, DerivativeNegativeDeterminantCases)
{
	using CubicSolver::evaluatePolynomial;
	using CubicSolver::isZero;

	double roots[3];
	int numberOfRoots;

	{
		SCOPED_TRACE("P(x) = x^3 + x^2 + x + 1 = 0 => P'(x) = 3x^2 + 2x + 1 => discriminant = 4 - 4*(3)*(1) = -8");
		// P(0) = 1
		// P(1) = 4
		// P is monotonic and 0 is not in [P(0)..P(1)], so the unique solution is not within [0..1]
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(1.0, 1.0, 1.0, 1.0, roots));
		EXPECT_EQ(0, numberOfRoots);
	}

	{
		SCOPED_TRACE("P(x) = x^3 - x^2 + x - 0.5 = 0 => P'(x) = 3x^2 - 2x + 1 => discriminant = 4 - 4*(3)*(1) = -8");
		// P(0) = -0.5
		// P(1) = 0.5
		// P is monotonic and 0 is in [P(0)..P(1)], so the unique solution is within [0..1]
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(1.0, -1.0, 1.0, -0.5, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_TRUE(roots[0] >= 0.0 && roots[0] <= 1.0);
		double eval = evaluatePolynomial(1.0, -1.0, 1.0, -0.5, roots[0]);
		EXPECT_TRUE(isZero(eval)) << "P(" << roots[0] << ") = " << eval;
	}
};

TEST(CubicSolverTests, DerivativePositiveDeterminantCases)
{
	using CubicSolver::evaluatePolynomial;
	using CubicSolver::isZero;

	double roots[3];
	int numberOfRoots;

	{
		SCOPED_TRACE("P(x) = -x^3 + x^2 + x + 1 = 0 => P'(x) = -3x^2 + 2x + 1 => discriminant = 4 - 4*(-3)*(1) = 16");
		// P' has 2 roots: -1/3 and 1
		// P'(-1/3) = 0                                         P'(1) = 0
		// P (-1/3) = 1 > -1/27 + 1/9 - 1/3 + 1 > 0   P(0) = 1  P (1) = 2
		// P is monotonic in 3 intervals [-Inf -1/3[, [-1/3 1] and ]1 +Inf[
		// Therefore P is monotonic in [0..1] and above 0 on this interval, so there is no root in [0..1]
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(-1.0, 1.0, 1.0, 1.0, roots));
		EXPECT_EQ(0, numberOfRoots);
	}

	{
		SCOPED_TRACE("P(x) = -x^3 + x^2 + x - 0.5 = 0 => P'(x) = -3x^2 + 2x + 1 => discriminant = 4 - 4*(-3)*(1) = 16");
		// P' has 2 roots: -1/3 and 1
		// P'(-1/3) = 0                                          P'(1) = 0
		// P (-1/3) = -1/27 + 1/9 - 1/3 -0.5 < 0   P(0) = -0.5   P (1) = 0.5 > 0
		// P is monotonic in 3 intervals [-Inf -1/3[, [-1/3 1] and ]1 +Inf[
		// Therefore P is monotonic in [0..1] and cross 0 on this interval, so there is 1 root in [0..1]
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(-1.0, 1.0, 1.0, -0.5, roots));
		EXPECT_EQ(1, numberOfRoots);
		EXPECT_TRUE(roots[0] >= 0.0 && roots[0] <= 1.0);
		double eval = evaluatePolynomial(-1.0, 1.0, 1.0, -0.5, roots[0]);
		EXPECT_TRUE(isZero(eval)) << "P(" << roots[0] << ") = " << eval;
	}

	{
		// From a real use case of point/triangle continuous collision detection
		SCOPED_TRACE("P(x) = 7.84x^3 - 37.08x^2 + 19.5x - 1.46 = 0 => P'(x) = 23.52x^2 - 74.16x + 19.5");
		// P' has 2 roots: x1 = (74.16 - 60.540445984482142729500176673066) / 47.04 = 0.2895313353639000270089248156236
		//             and x2 = (74.16 + 60.540445984482142729500176673066) / 47.04 = 2.8635298891258958913584221231519
		// P is monotonic in 3 intervals [-Inf x1[, [x1 x2] and ]x2 +Inf[
		// P'(0) > 0         ; P'(x1) = 0         ; P'(1) = -31.14     ; P'(x2) = 0
		// P(0) = -1.46 < 0  ; P (x1) = 1.26 > 0  ; P (1) = -11.2 < 0  ; P(x2) = -65.58
		// Therefore P is monotonic and has a solution in [0..x1] and [x1 1]
		EXPECT_NO_THROW(numberOfRoots = findRootsInRange01(7.84, -37.08, 19.5, -1.46, roots));
		EXPECT_EQ(2, numberOfRoots);
		EXPECT_TRUE(roots[0] >= 0.0 && roots[0] <= 1.0);
		EXPECT_TRUE(roots[1] >= 0.0 && roots[1] <= 1.0);
		EXPECT_TRUE(roots[1] > roots[0]);
		double eval = evaluatePolynomial(7.84, -37.08, 19.5, -1.46, roots[0]);
		EXPECT_TRUE(isZero(eval, 1e-15)) << "P(" << roots[0] << ") = " << eval;
		eval = evaluatePolynomial(7.84, -37.08, 19.5, -1.46, roots[1]);
		EXPECT_TRUE(isZero(eval, 1e-15)) << "P(" << roots[1] << ") = " << eval;
	}
};

}; // namespace Math
}; // namespace SurgSim

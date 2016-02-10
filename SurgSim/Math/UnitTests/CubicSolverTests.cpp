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
	bool found;

	{
		SCOPED_TRACE("0.x^3 + x^2 + x + 1 = 0 (no solution on R: none on [0..1])");
		EXPECT_NO_THROW(found = findSmallestRootInRange01(0.0, 1.0, 1.0, 1.0, roots));
		EXPECT_EQ(false, found);
	}

	{
		SCOPED_TRACE("0.x^3 + 1x^2 + 4x - 12 = 0 (2 solutions on R: -6 and 2, none on [0..1])");
		EXPECT_NO_THROW(found = findSmallestRootInRange01(0.0, 1.0, 4.0, -12.0, roots));
		EXPECT_EQ(false, found);
	}

	{
		SCOPED_TRACE("0.x^3 + x^2 + 0x + 0 = 0 (1 solution on R: 0, 1 on [0..1])");
		EXPECT_NO_THROW(found = findSmallestRootInRange01(0.0, 1.0, 0.0, 0.0, roots));
		EXPECT_EQ(true, found);
		EXPECT_DOUBLE_EQ(0.0, roots[0]);
	}

	{
		SCOPED_TRACE("0.x^3 + x^2 + x - 2 = 0 (2 solutions on R: -2 and 1, 1 on [0..1])");
		EXPECT_NO_THROW(found = findSmallestRootInRange01(0.0, 1.0, 1.0, -2.0, roots));
		EXPECT_EQ(true, found);
		EXPECT_DOUBLE_EQ(1.0, roots[0]);
	}

	{
		SCOPED_TRACE("0.x^3 + 2x^2 + x - 1 = 0 (2 solutions on R: -1 and 0.5, 1 on [0..1])");
		EXPECT_NO_THROW(found = findSmallestRootInRange01(0.0, 2.0, 1.0, -1.0, roots));
		EXPECT_EQ(true, found);
		EXPECT_DOUBLE_EQ(0.5, roots[0]);
	}

};

}; // namespace Math
}; // namespace SurgSim

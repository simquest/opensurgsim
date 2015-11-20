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
/// Tests for the Scalar functions.

#include <gtest/gtest.h>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Scalar.h"

namespace SurgSim
{

namespace Math
{

class ScalarTests : public ::testing::Test {};


TEST(ScalarTests, TwoEntriesTests)
{
	int minInt = 7;
	int maxInt = 52;
	int valueInt;
	int epsilonInt = 5;

	float minFloat = 7.0;
	float maxFloat = 52.0;
	float valueFloat;
	float epsilonFloat = 5.0;

	double minDouble = 7.0;
	double maxDouble = 52.0;
	double valueDouble;
	double epsilonDouble = 5.0;

	{
		SCOPED_TRACE("Test < minimum");

		valueInt = 6;
		EXPECT_EQ(minInt, Math::clamp(valueInt, minInt, maxInt, epsilonInt));

		valueFloat = 6.0;
		EXPECT_FLOAT_EQ(minFloat, Math::clamp(valueFloat, minFloat, maxFloat, epsilonFloat));

		valueDouble = 6.0;
		EXPECT_DOUBLE_EQ(minDouble, Math::clamp(valueDouble, minDouble, maxDouble, epsilonDouble));
	}

	{
		SCOPED_TRACE("Test value = minimum + epsilon");

		valueInt = 12;
		EXPECT_EQ(minInt, Math::clamp(valueInt, minInt, maxInt, epsilonInt));

		valueFloat = 12.0;
		EXPECT_FLOAT_EQ(minFloat, Math::clamp(valueFloat, minFloat, maxFloat, epsilonFloat));

		valueDouble = 12.0;
		EXPECT_DOUBLE_EQ(minDouble, Math::clamp(valueDouble, minDouble, maxDouble, epsilonDouble));
	}

	{
		SCOPED_TRACE("Test value > minimum + epsilon");

		valueInt = 13;
		EXPECT_EQ(13, Math::clamp(valueInt, minInt, maxInt, epsilonInt));

		valueFloat = static_cast<float>(12.0 + 1.0e-04);
		EXPECT_FLOAT_EQ(static_cast<float>(12.0 + 1.0e-04), Math::clamp(valueFloat, minFloat, maxFloat, epsilonFloat));

		valueDouble = 12.0 + 1.0e-12;
		EXPECT_DOUBLE_EQ(12.0 + 1.0e-12, Math::clamp(valueDouble, minDouble, maxDouble, epsilonDouble));
	}

	{
		SCOPED_TRACE("Test > maximum");

		valueInt = 54;
		EXPECT_EQ(maxInt, Math::clamp(valueInt, minInt, maxInt, epsilonInt));

		valueFloat = 54.0;
		EXPECT_FLOAT_EQ(maxFloat, Math::clamp(valueFloat, minFloat, maxFloat, epsilonFloat));

		valueDouble = 54.0;
		EXPECT_DOUBLE_EQ(maxDouble, Math::clamp(valueDouble, minDouble, maxDouble, epsilonDouble));
	}

	{
		SCOPED_TRACE("Test value = maximum - epsilon");

		valueInt = 47;
		EXPECT_EQ(maxInt, Math::clamp(valueInt, minInt, maxInt, epsilonInt));

		valueFloat = 47.0;
		EXPECT_FLOAT_EQ(maxFloat, Math::clamp(valueFloat, minFloat, maxFloat, epsilonFloat));

		valueDouble = 47.0;
		EXPECT_DOUBLE_EQ(maxDouble, Math::clamp(valueDouble, minDouble, maxDouble, epsilonDouble));
	}

	{
		SCOPED_TRACE("Test value < maximum - epsilon");

		valueInt = 46;
		EXPECT_EQ(46, Math::clamp(valueInt, minInt, maxInt, epsilonInt));

		valueFloat = static_cast<float>(47.0 - 1.0e-04);
		EXPECT_FLOAT_EQ(static_cast<float>(47.0 - 1.0e-04), Math::clamp(valueFloat, minFloat, maxFloat, epsilonFloat));

		valueDouble = 47.0 - 1.0e-12;
		EXPECT_DOUBLE_EQ(47.0 - 1.0e-12, Math::clamp(valueDouble, minDouble, maxDouble, epsilonDouble));
	}

	{
		SCOPED_TRACE("Test maximum - epsilon < value > minimum + epsilon ");

		valueInt = 36;
		epsilonInt = 30;
		EXPECT_EQ(maxInt, Math::clamp(valueInt, minInt, maxInt, epsilonInt));

		valueFloat = 36.0;
		epsilonFloat = 30;
		EXPECT_FLOAT_EQ(maxFloat, Math::clamp(valueFloat, minFloat, maxFloat, epsilonFloat));

		valueDouble = 36.0;
		epsilonDouble = 30.0;
		EXPECT_DOUBLE_EQ(maxDouble, Math::clamp(valueDouble, minDouble, maxDouble, epsilonDouble));
	}
	{
		SCOPED_TRACE("Test matrix implementation for all cases ");

		Math::Matrix33d m1 = Math::Matrix33d::Zero();
		m1(0, 0) = 12.0;
		m1(0, 1) = 6.0;
		m1(0, 2) = 12.0 + 1.0e-12;
		m1(1, 0) = 54.0;
		m1(1, 1) = 47.0;
		m1(1, 2) = 47.0 - 1.0e-12;
		m1(2, 0) = 36.0;

		Math::Matrix33d backup = m1;
		double epsilonDouble = 5.0;

		Math::Matrix33d result = m1.unaryExpr(clampOperator<double>(minDouble, maxDouble, epsilonDouble));
		for (size_t row = 0; row < 3; ++row)
		{
			for (size_t column = 0; column < 3; ++column)
			{
				EXPECT_DOUBLE_EQ(Math::clamp(backup.coeffRef(row, column), minDouble, maxDouble, epsilonDouble),
								 result.coeff(row, column));
			}
		}

		epsilonDouble = 30.0;

		result = m1.unaryExpr(clampOperator<double>(minDouble, maxDouble, epsilonDouble));
		for (size_t row = 0; row < 3; ++row)
		{
			for (size_t column = 0; column < 3; ++column)
			{
				EXPECT_DOUBLE_EQ(Math::clamp(backup.coeffRef(row, column), minDouble, maxDouble, epsilonDouble),
								 result.coeff(row, column));
			}
		}
	}
};

}; // namespace Math

}; // namespace SurgSim

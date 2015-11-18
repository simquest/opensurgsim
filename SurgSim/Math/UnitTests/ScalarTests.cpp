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
		Math::epsilonClamp(&valueInt, minInt, maxInt, epsilonInt);
		EXPECT_EQ(minInt, valueInt);

		valueFloat = 6.0;
		Math::epsilonClamp(&valueFloat, minFloat, maxFloat, epsilonFloat);
		EXPECT_FLOAT_EQ(minFloat, valueFloat);

		valueDouble = 6.0;
		Math::epsilonClamp(&valueDouble, minDouble, maxDouble, epsilonDouble);
		EXPECT_DOUBLE_EQ(minDouble, valueDouble);
	}

	{
		SCOPED_TRACE("Test value = minimum + epsilon");

		valueInt = 12;
		Math::epsilonClamp(&valueInt, minInt, maxInt, epsilonInt);
		EXPECT_EQ(minInt, valueInt);

		valueFloat = 12.0;
		Math::epsilonClamp(&valueFloat, minFloat, maxFloat, epsilonFloat);
		EXPECT_FLOAT_EQ(minFloat, valueFloat);

		valueDouble = 12.0;
		Math::epsilonClamp(&valueDouble, minDouble, maxDouble, epsilonDouble);
		EXPECT_DOUBLE_EQ(minDouble, valueDouble);
	}

	{
		SCOPED_TRACE("Test value > minimum + epsilon");

		valueInt = 13;
		Math::epsilonClamp(&valueInt, minInt, maxInt, epsilonInt);
		EXPECT_EQ(13, valueInt);

		valueFloat = static_cast<float>(12.0 + 1.0e-04);
		Math::epsilonClamp(&valueFloat, minFloat, maxFloat, epsilonFloat);
		EXPECT_FLOAT_EQ(static_cast<float>(12.0 + 1.0e-04), valueFloat);

		valueDouble = 12.0 + 1.0e-12;
		Math::epsilonClamp(&valueDouble, minDouble, maxDouble, epsilonDouble);
		EXPECT_DOUBLE_EQ(12.0 + 1.0e-12, valueDouble);
	}

	{
		SCOPED_TRACE("Test > maximum");

		valueInt = 54;
		Math::epsilonClamp(&valueInt, minInt, maxInt, epsilonInt);
		EXPECT_EQ(maxInt, valueInt);

		valueFloat = 54.0;
		Math::epsilonClamp(&valueFloat, minFloat, maxFloat, epsilonFloat);
		EXPECT_FLOAT_EQ(maxFloat, valueFloat);

		valueDouble = 54.0;
		Math::epsilonClamp(&valueDouble, minDouble, maxDouble, epsilonDouble);
		EXPECT_DOUBLE_EQ(maxDouble, valueDouble);
	}

	{
		SCOPED_TRACE("Test value = maximum - epsilon");

		valueInt = 47;
		Math::epsilonClamp(&valueInt, minInt, maxInt, epsilonInt);
		EXPECT_EQ(maxInt, valueInt);

		valueFloat = 47.0;
		Math::epsilonClamp(&valueFloat, minFloat, maxFloat, epsilonFloat);
		EXPECT_FLOAT_EQ(maxFloat, valueFloat);

		valueDouble = 47.0;
		Math::epsilonClamp(&valueDouble, minDouble, maxDouble, epsilonDouble);
		EXPECT_DOUBLE_EQ(maxDouble, valueDouble);
	}

	{
		SCOPED_TRACE("Test value < maximum - epsilon");

		valueInt = 46;
		Math::epsilonClamp(&valueInt, minInt, maxInt, epsilonInt);
		EXPECT_EQ(46, valueInt);

		valueFloat = static_cast<float>(47.0 - 1.0e-04);
		Math::epsilonClamp(&valueFloat, minFloat, maxFloat, epsilonFloat);
		EXPECT_FLOAT_EQ(static_cast<float>(47.0 - 1.0e-04), valueFloat);

		valueDouble = 47.0 - 1.0e-12;
		Math::epsilonClamp(&valueDouble, minDouble, maxDouble, epsilonDouble);
		EXPECT_DOUBLE_EQ(47.0 - 1.0e-12, valueDouble);
	}

	{
		SCOPED_TRACE("Test maximum - epsilon < value > minimum + epsilon ");

		valueInt = 36;
		epsilonInt = 30;
		Math::epsilonClamp(&valueInt, minInt, maxInt, epsilonInt);
		EXPECT_EQ(maxInt, valueInt);

		valueFloat = 36.0;
		epsilonFloat = 30;
		Math::epsilonClamp(&valueFloat, minFloat, maxFloat, epsilonFloat);
		EXPECT_FLOAT_EQ(maxFloat, valueFloat);

		valueDouble = 36.0;
		epsilonDouble = 30.0;
		Math::epsilonClamp(&valueDouble, minDouble, maxDouble, epsilonDouble);
		EXPECT_DOUBLE_EQ(maxDouble, valueDouble);
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

		Math::Matrix33d result = m1.unaryExpr(CwiseEpsilonClampOp<double>(minDouble, maxDouble, epsilonDouble));
		for (size_t row = 0; row < 3; ++row)
		{
			for (size_t column = 0; column < 3; ++column)
			{
				Math::epsilonClamp(&backup.coeffRef(row, column), minDouble, maxDouble, epsilonDouble);
				EXPECT_DOUBLE_EQ(backup.coeff(row, column), result.coeff(row, column));
			}
		}

		backup = m1;
		epsilonDouble = 30.0;

		result = m1.unaryExpr(CwiseEpsilonClampOp<double>(minDouble, maxDouble, epsilonDouble));
		for (size_t row = 0; row < 3; ++row)
		{
			for (size_t column = 0; column < 3; ++column)
			{
				Math::epsilonClamp(&backup.coeffRef(row, column), minDouble, maxDouble, epsilonDouble);
				EXPECT_DOUBLE_EQ(backup.coeff(row, column), result.coeff(row, column));
			}
		}
	}
};

}; // namespace Math

}; // namespace SurgSim

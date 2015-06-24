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
/// Tests for the MinMax functions.

#include <gtest/gtest.h>

#include "SurgSim/Math/MinMax.h"

namespace SurgSim
{

namespace Math
{

class MinMaxTests : public ::testing::Test {};


TEST_F(MinMaxTests, TwoEntriesTests)
{
	float minFloatVal = 7;
	float maxFloatVal = 52;

	float floatMin;
	float floatMax;

	double minDoubleVal = -47;
	double maxDoubleVal = -2;
	double doubleMin;
	double doubleMax;

	minMax(minFloatVal, maxFloatVal, &floatMin, &floatMax);
	EXPECT_EQ(minFloatVal, floatMin);
	EXPECT_EQ(maxFloatVal, floatMax);

	minMax(maxFloatVal, minFloatVal, &floatMin, &floatMax);
	EXPECT_EQ(minFloatVal, floatMin);
	EXPECT_EQ(maxFloatVal, floatMax);

	minMax(minDoubleVal, maxDoubleVal, &doubleMin, &doubleMax);
	EXPECT_EQ(minDoubleVal, doubleMin);
	EXPECT_EQ(maxDoubleVal, doubleMax);

	minMax(maxDoubleVal, minDoubleVal, &doubleMin, &doubleMax);
	EXPECT_EQ(minDoubleVal, doubleMin);
	EXPECT_EQ(maxDoubleVal, doubleMax);
};

TEST_F(MinMaxTests, ThreeEntriesTests)
{
	float minFloatVal = 7;
	float maxFloatVal = 52;
	float midFloatVal = 22;
	float floatMin;
	float floatMax;

	double minDoubleVal = -47;
	double maxDoubleVal = -2;
	double midDoubleVal = -7;
	double doubleMin;
	double doubleMax;

	minMax(minFloatVal, maxFloatVal, midFloatVal, &floatMin, &floatMax);
	EXPECT_EQ(minFloatVal, floatMin);
	EXPECT_EQ(maxFloatVal, floatMax);

	minMax(maxFloatVal, minFloatVal, midFloatVal, &floatMin, &floatMax);
	EXPECT_EQ(minFloatVal, floatMin);
	EXPECT_EQ(maxFloatVal, floatMax);

	minMax(maxFloatVal, midFloatVal, minFloatVal, &floatMin, &floatMax);
	EXPECT_EQ(minFloatVal, floatMin);
	EXPECT_EQ(maxFloatVal, floatMax);

	minMax(minFloatVal, midFloatVal, maxFloatVal, &floatMin, &floatMax);
	EXPECT_EQ(minFloatVal, floatMin);
	EXPECT_EQ(maxFloatVal, floatMax);

	minMax(midFloatVal, maxFloatVal, minFloatVal, &floatMin, &floatMax);
	EXPECT_EQ(minFloatVal, floatMin);
	EXPECT_EQ(maxFloatVal, floatMax);

	minMax(midFloatVal, minFloatVal, maxFloatVal, &floatMin, &floatMax);
	EXPECT_EQ(minFloatVal, floatMin);
	EXPECT_EQ(maxFloatVal, floatMax);

	minMax(minDoubleVal, maxDoubleVal, midDoubleVal, &doubleMin, &doubleMax);
	EXPECT_EQ(minDoubleVal, doubleMin);
	EXPECT_EQ(maxDoubleVal, doubleMax);

	minMax(maxDoubleVal, minDoubleVal, midDoubleVal, &doubleMin, &doubleMax);
	EXPECT_EQ(minDoubleVal, doubleMin);
	EXPECT_EQ(maxDoubleVal, doubleMax);

	minMax(maxDoubleVal, midDoubleVal, minDoubleVal, &doubleMin, &doubleMax);
	EXPECT_EQ(minDoubleVal, doubleMin);
	EXPECT_EQ(maxDoubleVal, doubleMax);

	minMax(minDoubleVal, midDoubleVal, maxDoubleVal, &doubleMin, &doubleMax);
	EXPECT_EQ(minDoubleVal, doubleMin);
	EXPECT_EQ(maxDoubleVal, doubleMax);

	minMax(midDoubleVal, maxDoubleVal, minDoubleVal, &doubleMin, &doubleMax);
	EXPECT_EQ(minDoubleVal, doubleMin);
	EXPECT_EQ(maxDoubleVal, doubleMax);

	minMax(midDoubleVal, minDoubleVal, maxDoubleVal, &doubleMin, &doubleMax);
	EXPECT_EQ(minDoubleVal, doubleMin);
	EXPECT_EQ(maxDoubleVal, doubleMax);
};

TEST_F(MinMaxTests, FourEntriesTests)
{
	const int numValues = 4;

	float floatValues[numValues] = { 18, 7, 52, 12};
	float minFloatVal = 7;
	float maxFloatVal = 52;
	float floatMin;
	float floatMax;

	double doubleValues[numValues] = { -23, -47, -2, -17};
	double minDoubleVal = -47;
	double maxDoubleVal = -2;
	double doubleMin;
	double doubleMax;

	for (int i1 = 0; i1 < numValues; ++i1)
	{
		for (int i2 = 0; i2 < numValues; ++i2)
		{
			if (i2 != i1)
			{
				for (int i3 = 0; i3 < numValues; ++i3)
				{
					if ((i3 != i1) && (i3 != i2))
					{
						for (int i4 = 0; i4 < numValues; ++i4)
						{
							if ((i4 != i1) && (i4 != i2) && (i4 != i3))
							{
								minMax(floatValues[i1], floatValues[i2], floatValues[i3],
									   floatValues[i4], &floatMin, &floatMax);
								EXPECT_EQ(minFloatVal, floatMin);
								EXPECT_EQ(maxFloatVal, floatMax);

								minMax(doubleValues[i1], doubleValues[i2], doubleValues[i3],
									   doubleValues[i4], &doubleMin, &doubleMax);
								EXPECT_EQ(minDoubleVal, doubleMin);
								EXPECT_EQ(maxDoubleVal, doubleMax);
							}
						}
					}
				}
			}
		}
	}
};

TEST_F(MinMaxTests, FiveEntriesTests)
{
	const int numValues = 5;

	float floatValues[numValues] = { 18, 7, 52, 12, 12};
	float minFloatVal = 7;
	float maxFloatVal = 52;
	float floatMin;
	float floatMax;

	double doubleValues[numValues] = { -23, -47, -2, -17, -17};
	double minDoubleVal = -47;
	double maxDoubleVal = -2;
	double doubleMin;
	double doubleMax;

	for (int i1 = 0; i1 < numValues; ++i1)
	{
		for (int i2 = 0; i2 < numValues; ++i2)
		{
			if (i2 != i1)
			{
				for (int i3 = 0; i3 < numValues; ++i3)
				{
					if ((i3 != i1) && (i3 != i2))
					{
						for (int i4 = 0; i4 < numValues; ++i4)
						{
							if ((i4 != i1) && (i4 != i2) && (i4 != i3))
							{
								for (int i5 = 0; i5 < numValues; ++i5)
								{
									if ((i5 != i1) && (i5 != i2) && (i5 != i3) && (i5 != i4))
									{
										minMax(floatValues[i1], floatValues[i2], floatValues[i3],
											   floatValues[i4], floatValues[i5], &floatMin, &floatMax);
										EXPECT_EQ(minFloatVal, floatMin);
										EXPECT_EQ(maxFloatVal, floatMax);

										minMax(doubleValues[i1], doubleValues[i2], doubleValues[i3],
											   doubleValues[i4], doubleValues[i5], &doubleMin, &doubleMax);
										EXPECT_EQ(minDoubleVal, doubleMin);
										EXPECT_EQ(maxDoubleVal, doubleMax);
									}
								}
							}
						}
					}
				}
			}
		}
	}
};

TEST_F(MinMaxTests, AribtraryEntriesTests)
{
	const int numValues = 5;

	float floatValues[numValues] = { 18, 7, 52, 12, 12};
	float minFloatVal = 7;
	float maxFloatVal = 52;
	float floatMin;
	float floatMax;

	double doubleValues[numValues] = { -23, -47, -2, -17, -17};
	double minDoubleVal = -47;
	double maxDoubleVal = -2;
	double doubleMin;
	double doubleMax;

	ASSERT_THROW(minMax(floatValues, 0, &floatMin, &floatMax), SurgSim::Framework::AssertionFailure);
	ASSERT_THROW(minMax(floatValues, -1, &floatMin, &floatMax), SurgSim::Framework::AssertionFailure);

	minMax(floatValues, numValues, &floatMin, &floatMax);
	EXPECT_EQ(minFloatVal, floatMin);
	EXPECT_EQ(maxFloatVal, floatMax);

	ASSERT_THROW(minMax(doubleValues, 0, &doubleMin, &doubleMax), SurgSim::Framework::AssertionFailure);
	ASSERT_THROW(minMax(doubleValues, -1, &doubleMin, &doubleMax), SurgSim::Framework::AssertionFailure);

	minMax(doubleValues, numValues, &doubleMin, &doubleMax);
	EXPECT_EQ(minDoubleVal, doubleMin);
	EXPECT_EQ(maxDoubleVal, doubleMax);
};

}; // namespace Math

}; // namespace SurgSim

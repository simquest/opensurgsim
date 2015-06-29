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
	const int numValues = 2;

	float floatValues[numValues] = { 7, 52};
	float minFloatVal = 7;
	float maxFloatVal = 52;
	float floatMin;
	float floatMax;

	double doubleValues[numValues] = { -47, -2};
	double minDoubleVal = -47;
	double maxDoubleVal = -2;
	double doubleMin;
	double doubleMax;

	std::sort(floatValues, floatValues + numValues);
	do
	{
		minMax(floatValues[0], floatValues[1], &floatMin, &floatMax);
		EXPECT_EQ(minFloatVal, floatMin);
		EXPECT_EQ(maxFloatVal, floatMax);
	}
	while (std::next_permutation(floatValues, floatValues + numValues));

	std::sort(doubleValues, doubleValues + numValues);
	do
	{
		minMax(doubleValues[0], doubleValues[1], &doubleMin, &doubleMax);
		EXPECT_EQ(minDoubleVal, doubleMin);
		EXPECT_EQ(maxDoubleVal, doubleMax);
	}
	while (std::next_permutation(doubleValues, doubleValues + numValues));
};

TEST_F(MinMaxTests, ThreeEntriesTests)
{
	const int numValues = 3;

	float floatValues[numValues] = { 22, 7, 52};
	float minFloatVal = 7;
	float maxFloatVal = 52;
	float floatMin;
	float floatMax;

	double doubleValues[numValues] = { -47, -2, -7};
	double minDoubleVal = -47;
	double maxDoubleVal = -2;
	double doubleMin;
	double doubleMax;

	std::sort(floatValues, floatValues + numValues);
	do
	{
		minMax(floatValues[0], floatValues[1], floatValues[2], &floatMin, &floatMax);
		EXPECT_EQ(minFloatVal, floatMin);
		EXPECT_EQ(maxFloatVal, floatMax);
	}
	while (std::next_permutation(floatValues, floatValues + numValues));

	std::sort(doubleValues, doubleValues + numValues);
	do
	{
		minMax(doubleValues[0], doubleValues[1], doubleValues[2], &doubleMin, &doubleMax);
		EXPECT_EQ(minDoubleVal, doubleMin);
		EXPECT_EQ(maxDoubleVal, doubleMax);
	}
	while (std::next_permutation(doubleValues, doubleValues + numValues));
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

	std::sort(floatValues, floatValues + numValues);
	do
	{
		minMax(floatValues[0], floatValues[1], floatValues[2], floatValues[3], &floatMin, &floatMax);
		EXPECT_EQ(minFloatVal, floatMin);
		EXPECT_EQ(maxFloatVal, floatMax);
	}
	while (std::next_permutation(floatValues, floatValues + numValues));

	std::sort(doubleValues, doubleValues + numValues);
	do
	{
		minMax(doubleValues[0], doubleValues[1], doubleValues[2], doubleValues[3], &doubleMin, &doubleMax);
		EXPECT_EQ(minDoubleVal, doubleMin);
		EXPECT_EQ(maxDoubleVal, doubleMax);
	}
	while (std::next_permutation(doubleValues, doubleValues + numValues));
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

	std::sort(floatValues, floatValues + numValues);
	do
	{
		minMax(floatValues[0], floatValues[1], floatValues[2],
			   floatValues[3], floatValues[4], &floatMin, &floatMax);
		EXPECT_EQ(minFloatVal, floatMin);
		EXPECT_EQ(maxFloatVal, floatMax);
	}
	while (std::next_permutation(floatValues, floatValues + numValues));

	std::sort(doubleValues, doubleValues + numValues);
	do
	{
		minMax(doubleValues[0], doubleValues[1], doubleValues[2],
			   doubleValues[3], doubleValues[4], &doubleMin, &doubleMax);
		EXPECT_EQ(minDoubleVal, doubleMin);
		EXPECT_EQ(maxDoubleVal, doubleMax);
	}
	while (std::next_permutation(doubleValues, doubleValues + numValues));
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

	std::sort(floatValues, floatValues + numValues);
	do
	{
		minMax(floatValues, numValues, &floatMin, &floatMax);
		EXPECT_EQ(minFloatVal, floatMin);
		EXPECT_EQ(maxFloatVal, floatMax);
	}
	while (std::next_permutation(floatValues, floatValues + numValues));

	ASSERT_THROW(minMax(doubleValues, 0, &doubleMin, &doubleMax), SurgSim::Framework::AssertionFailure);
	ASSERT_THROW(minMax(doubleValues, -1, &doubleMin, &doubleMax), SurgSim::Framework::AssertionFailure);

	std::sort(doubleValues, doubleValues + numValues);
	do
	{
		minMax(doubleValues, numValues, &doubleMin, &doubleMax);
		EXPECT_EQ(minDoubleVal, doubleMin);
		EXPECT_EQ(maxDoubleVal, doubleMax);
	}
	while (std::next_permutation(doubleValues, doubleValues + numValues));
};

}; // namespace Math

}; // namespace SurgSim

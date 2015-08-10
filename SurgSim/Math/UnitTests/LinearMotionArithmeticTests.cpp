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
/// Tests for the LinearMotionArithmetic functions.

#include <array>

#include <gtest/gtest.h>

#include "SurgSim/Math/LinearMotionArithmetic.h"

namespace SurgSim
{

namespace Math
{

namespace
{
double epsilon = 1.0e-10;
}

template <typename T>
class LinearMotionArithmeticTests : public ::testing::Test
{
public:
	typedef LinearMotionND<double, 2> LinearMotionDouble2;
	typedef LinearMotionND<double, 3> LinearMotionDouble3;

	LinearMotion<double> testLinearMotionMoveConstructor(LinearMotion<double> dummy)
	{
		LinearMotion<double> ret;
		ret = dummy;
		return ret;
	}

	template <int dimension>
	LinearMotionND<double, dimension> testLinearMotionNDMoveConstructor(LinearMotionND<double, dimension> dummy)
	{
		LinearMotionND<double, dimension> ret;
		ret = dummy;
		return ret;
	}

	void initializeConstructor(LinearMotion<double>* motion)
	{
		// Constructor tests
		EXPECT_NO_THROW(LinearMotion<double> a);
		EXPECT_NO_THROW(LinearMotion<double> a(1.0, 2.0));
		EXPECT_NO_THROW(LinearMotion<double> a(2.0, 1.0));
		LinearMotion<double> a(2.0, 1.0);
		EXPECT_NO_THROW(LinearMotion<double> b(a));
		LinearMotion<double> b(a);
		EXPECT_TRUE(b == a);
		EXPECT_NO_THROW(testLinearMotionMoveConstructor(b));
		LinearMotion<double> f = testLinearMotionMoveConstructor(b);
		EXPECT_TRUE(f == b);
		*motion = a;
	}

	template <int dimension>
	void initializeConstructor(LinearMotionND<double, dimension>* motion)
	{
		typedef LinearMotionND<double, dimension> LinearMotionDoubleType;

		// Constructor tests
		LinearMotion<double> testLinearMotion(3.8, 3.7);
		std::array<double, dimension> starts;
		std::array<double, dimension> ends;
		std::array<LinearMotion<double>, dimension> testLinearMotionArray;
		for (int counter = 0; counter < dimension; ++counter)
		{
			starts[counter] = counter + 1.0;
			ends[counter] = counter + 2.0;
			testLinearMotionArray[counter] = LinearMotion<double> (counter + 2.0, counter + 1.0);
		}

		EXPECT_NO_THROW(LinearMotionDoubleType a);
		EXPECT_NO_THROW(LinearMotionDoubleType a(testLinearMotionArray));
		EXPECT_NO_THROW(LinearMotionDoubleType a(starts, ends));
		EXPECT_NO_THROW(LinearMotionDoubleType a(LinearMotionDoubleType(starts, ends)));

		LinearMotionDoubleType b(testLinearMotionArray);
		LinearMotionDoubleType a(b);
		EXPECT_TRUE(a == b);
		EXPECT_NO_THROW(testLinearMotionNDMoveConstructor(b));
		LinearMotionDoubleType f = testLinearMotionNDMoveConstructor(b);
		EXPECT_TRUE(f == b);
		*motion = a;
	}

	void checkLinearMotionConstructor(LinearMotion<double>* motion)
	{
		EXPECT_DOUBLE_EQ(2.0, motion->getStart());
		EXPECT_DOUBLE_EQ(1.0, motion->getEnd());
	}

	template <int dimension>
	void checkLinearMotionConstructor(LinearMotionND<double, dimension>* motion)
	{
		// Check get ... up to numb
		for (int counter = 0; counter < dimension; ++counter)
		{
			EXPECT_DOUBLE_EQ(counter + 2.0, motion->getAxis(counter).getStart());
			EXPECT_DOUBLE_EQ(counter + 1.0, motion->getAxis(counter).getEnd());
		}
		EXPECT_THROW(motion->getAxis(-1), SurgSim::Framework::AssertionFailure);
		EXPECT_THROW(motion->getAxis(dimension), SurgSim::Framework::AssertionFailure);
	}

	void setLinearMotionFromOffset(double offsetStart, double offsetEnd, LinearMotion<double>* motion)
	{
		(*motion) = LinearMotion<double>(offsetStart, offsetEnd);
	}

	template <int dimension>
	void setLinearMotionFromOffset(double offsetStart, double offsetEnd, LinearMotionND<double, dimension>* motion)
	{
		std::array<LinearMotion<double>, dimension> values;
		for (size_t counter = 0; counter < dimension; ++counter)
		{
			values[counter] = LinearMotion<double>(counter + offsetStart, counter + offsetEnd);
		}
		(*motion) = LinearMotionND<double, dimension>(values);
	}

	void checkLinearMotionOperators(LinearMotion<double>* motion1, LinearMotion<double>* motion2)
	{
		// Assignment and equal/not equal tests
		LinearMotion<double> c;
		c = *motion2;
		EXPECT_TRUE(c != *motion1);
		EXPECT_FALSE(c == *motion1);
		c = *motion1;
		EXPECT_FALSE(c != *motion1);
		EXPECT_TRUE(c == *motion1);

		// Move assignment
		LinearMotion<double> e;
		LinearMotion<double> zeros(0.0, 0.0);
		e = (*motion1 + zeros);
		EXPECT_TRUE(e == *motion1);

		// to interval and polynomial
		EXPECT_TRUE(motion1->toInterval().isApprox(Interval<double>(1.0, 2.0), epsilon));
		EXPECT_TRUE(motion1->toPolynomial().isApprox(Polynomial<double, 1>(2.0, -1.0), epsilon));

		// Contains 0 and isApprox
		LinearMotion<double> containsZero(-1.0, 1.0);
		EXPECT_TRUE(containsZero.containsZero());
		EXPECT_FALSE(motion1->containsZero());
		EXPECT_FALSE(motion2->containsZero());
		LinearMotion<double> closeStart(motion1->getStart() + epsilon, motion1->getEnd());
		LinearMotion<double> closeEnd(motion1->getStart(), motion1->getEnd() + epsilon);
		EXPECT_TRUE(motion1->isApprox(closeStart, 2 * epsilon));
		EXPECT_TRUE(motion1->isApprox(closeEnd, 2 * epsilon));
		EXPECT_FALSE(motion1->isApprox(closeStart, 0.5 * epsilon));
		EXPECT_FALSE(motion1->isApprox(closeEnd, 0.5 * epsilon));

		// +, +=, -, -=, *, /
		LinearMotion<double> scratch;
		scratch = *motion1 + *motion2;
		EXPECT_TRUE(scratch.isApprox(LinearMotion<double>(3.0, 3.0), epsilon));
		scratch = *motion1;
		scratch += *motion2;
		EXPECT_TRUE(scratch.isApprox(LinearMotion<double>(3.0, 3.0), epsilon));
		scratch = *motion1 - *motion2;
		EXPECT_TRUE(scratch.isApprox(LinearMotion<double>(1.0, -1.0), epsilon));
		scratch = *motion1;
		scratch -= *motion2;
		EXPECT_TRUE(scratch.isApprox(LinearMotion<double>(1.0, -1.0), epsilon));
		EXPECT_TRUE((*motion1 * *motion2).isApprox(Interval<double>(1.0, 4.0), epsilon));
		EXPECT_TRUE((*motion1 / *motion2).isApprox(Interval<double>(0.5, 2.0), epsilon));

		// Range operators
		EXPECT_DOUBLE_EQ(1.75, motion1->atTime(0.25));
		EXPECT_TRUE(motion1->firstHalf().isApprox(LinearMotion<double>(2.0, 1.5), epsilon));
		EXPECT_TRUE(motion1->secondHalf().isApprox(LinearMotion<double>(1.5, 1.0), epsilon));
	}

	template <int dimension>
	void checkLinearMotionOperators(LinearMotionND<double, dimension>* motion1,
									LinearMotionND<double, dimension>* motion2)
	{
		typedef LinearMotionND<double, dimension> LinearMotionDoubleType;
		typedef IntervalND<double, dimension> IntervalDoubleType;

		// Assignment and equal/not equal tests
		LinearMotionDoubleType c;
		c = *motion2;
		EXPECT_TRUE(c != *motion1);
		EXPECT_FALSE(c == *motion1);
		c = *motion1;
		EXPECT_FALSE(c != *motion1);
		EXPECT_TRUE(c == *motion1);

		// Move assignment
		LinearMotionDoubleType e;
		std::array<double, dimension> zeroArray;
		for (int counter = 0; counter < dimension; ++counter)
		{
			zeroArray[counter] = 0.0;
		}
		LinearMotionDoubleType zeros(zeroArray, zeroArray);
		e = (*motion1 + zeros);
		EXPECT_TRUE(e == *motion1);

		// to interval
		std::array<Interval<double>, dimension> motionArray;
		for (int counter = 0; counter < dimension; ++counter)
		{
			motionArray[counter] = motion1->getAxis(counter).toInterval();
		}
		EXPECT_TRUE(motion1->toInterval().isApprox(IntervalDoubleType(motionArray), epsilon));

		// is approx
		std::array<Interval<double>, dimension> approxArray;
		for (int counter = 0; counter < dimension; ++counter)
		{
			approxArray[counter] = motion1->getAxis(counter).toInterval();
		}
		for (int counter = 0; counter < dimension; ++counter)
		{
			approxArray[counter] = (motion1->getAxis(counter) + LinearMotion<double>(epsilon, 0.0)).toInterval();
			EXPECT_TRUE(motion1->toInterval().isApprox(IntervalDoubleType(approxArray), 2.0 * epsilon));
			EXPECT_FALSE(motion1->toInterval().isApprox(IntervalDoubleType(approxArray), epsilon / 2.0));
			approxArray[counter] = motion1->getAxis(counter).toInterval();
		}

		// +, +=, -, -=, *, /
		LinearMotionDoubleType scratch;
		scratch = *motion1 + *motion2;
		for (int counter = 0; counter < dimension; ++counter)
		{
			EXPECT_TRUE(scratch.getAxis(counter).isApprox(LinearMotion<double>((2.0 * counter) + 3.0,
						(2.0 * counter) + 3.0), epsilon));
		}
		scratch = *motion1;
		scratch += *motion2;
		for (int counter = 0; counter < dimension; ++counter)
		{
			EXPECT_TRUE(scratch.getAxis(counter).isApprox(LinearMotion<double>((2.0 * counter) + 3.0,
						(2.0 * counter) + 3.0), epsilon));
		}

		scratch = *motion1 - *motion2;
		for (int counter = 0; counter < dimension; ++counter)
		{
			EXPECT_TRUE(scratch.getAxis(counter).isApprox(LinearMotion<double>(1.0, -1.0), epsilon));
		}
		scratch = *motion1;
		scratch -= *motion2;
		for (int counter = 0; counter < dimension; ++counter)
		{
			EXPECT_TRUE(scratch.getAxis(counter).isApprox(LinearMotion<double>(1.0, -1.0), epsilon));
		}

		IntervalDoubleType intervalScratch;
		intervalScratch = *motion1 * *motion2;
		for (int counter = 0; counter < dimension; ++counter)
		{
			EXPECT_TRUE(intervalScratch.getAxis(counter).isApprox(Interval<double>((counter + 1) * (counter + 1),
						(counter + 2) * (counter + 2)), epsilon));
		}

		intervalScratch = *motion1 / *motion2;
		for (int counter = 0; counter < dimension; ++counter)
		{
			EXPECT_TRUE(intervalScratch.getAxis(counter).isApprox(
							Interval<double>((counter + 1.0) / (counter + 2.0),
											 (counter + 2.0) / (counter + 1.0)), epsilon));
		}

		// Range operators
		for (int counter = 0; counter < dimension; ++counter)
		{
			EXPECT_TRUE(motion1->firstHalf().getAxis(counter).isApprox(
							LinearMotion<double>(counter + 2.0, counter + 1.5), epsilon));
			EXPECT_TRUE(motion1->secondHalf().getAxis(counter).isApprox(
							LinearMotion<double>(counter + 1.5, counter + 1.0), epsilon));
		}
	}
};

typedef ::testing::Types <LinearMotion<double>,
		LinearMotionND<double, 2>,
		LinearMotionND<double, 3>> LinearMotionArithmeticTypes;

TYPED_TEST_CASE(LinearMotionArithmeticTests, LinearMotionArithmeticTypes);

TYPED_TEST(LinearMotionArithmeticTests, LinearMotionInitializationTests)
{
	// Constructor tests
	EXPECT_NO_THROW(TypeParam motion);
	TypeParam motion;
	this->initializeConstructor(&motion);
	this->checkLinearMotionConstructor(&motion);
};

TYPED_TEST(LinearMotionArithmeticTests, LinearMotionOperatorTests)
{
	TypeParam motion1;
	TypeParam motion2;
	this->setLinearMotionFromOffset(2, 1, &motion1);
	this->setLinearMotionFromOffset(1, 2, &motion2);
	this->checkLinearMotionOperators(&motion1, &motion2);
};

TEST(LinearMotionSpecializations, LinearMotionNDExtras)
{
	// Dot product
	std::array<double, 2> high;
	high[0] = 2.0;
	high[1] = 3.0;
	std::array<double, 2> low;
	low[0] = 1.0;
	low[1] = 2.0;
	LinearMotionND<double, 2> motion1(high, low);
	LinearMotionND<double, 2> motion2(low, high);

	double lower = 0;
	double upper = 0;
	for (int counter = 0; counter < 2; ++counter)
	{
		lower += (counter + 1) * (counter + 1);
		upper += (counter + 2) * (counter + 2);
	}
	EXPECT_TRUE(motion1.dotProduct(motion2).isApprox(Interval<double>(lower, upper), epsilon));
}

TEST(LinearMotionSpecializations, LinearMotion3DExtras)
{
	// Dot product
	std::array<double, 3> high;
	high[0] = 2.0;
	high[1] = 3.0;
	high[2] = 4.0;
	std::array<double, 3> low;
	low[0] = 1.0;
	low[1] = 2.0;
	low[2] = 3.0;
	LinearMotionND<double, 3> motion1(high, low);
	LinearMotionND<double, 3> motion2(low, high);
	Interval<double> range(0.0, 1.0);

	// Dot product
	// Dot product has an optimization using polynomials. Each linear motion is first converted to a linear
	// polynomial and then the 6 linear polynomials are multiplied against each other to give three quadratic
	// polynomials. The extrema of the sum of the quadratic polynomials limited to the interval under
	// evaluation form the minimum and maximum of the new interval. For this test, we end up with a parabola
	// opening downward. At 0 and 1 it has value 20. The maximum value occurs at 0.5 and has the value 20.75.
	EXPECT_TRUE(motion1.dotProduct(motion2, range).isApprox(Interval<double>(20.0, 20.75), epsilon));

	// Cross product
	IntervalND<double, 3> cross = motion1.crossProduct(motion2, range);
	EXPECT_TRUE(cross.getAxis(0).isApprox(Interval<double>(-1.0, 1.0), epsilon));
	EXPECT_TRUE(cross.getAxis(1).isApprox(Interval<double>(-2.0, 2.0), epsilon));
	EXPECT_TRUE(cross.getAxis(2).isApprox(Interval<double>(-1.0, 1.0), epsilon));

	/// Magnitudes
	EXPECT_TRUE(motion1.magnitudeSquared(range).isApprox(motion1.dotProduct(motion1, range), epsilon));
	Interval<double> magnitude = motion1.magnitude(range);
	Interval<double> dot = motion1.dotProduct(motion1, range);
	EXPECT_DOUBLE_EQ(std::sqrt(dot.getMin()), magnitude.getMin());
	EXPECT_DOUBLE_EQ(std::sqrt(dot.getMax()), magnitude.getMax());

	// At Time
	for (int counter = 0; counter < 3; ++counter)
	{
		EXPECT_DOUBLE_EQ(counter + 1.75, motion1.atTime(0.25)[counter]);
	}
}

TEST(LinearMotionUtilities, LinearMotionUtilities)
{
	LinearMotion<double> motion(-12.1, 8.7);

	// Output
	std::ostringstream linearMotionOutput;
	linearMotionOutput << motion;
	EXPECT_EQ("(-12.1 -> 8.7)", linearMotionOutput.str());
}

TEST(LinearMotionUtilities, LinearMotion3DUtilities)
{
	std::array<double, 3> high;
	high[0] = 3.0;
	high[1] = 4.0;
	high[2] = 5.0;
	std::array<double, 3> low;
	low[0] = 1.0;
	low[1] = 2.0;
	low[2] = 3.0;
	std::array<double, 3> medium;
	medium[0] = 2.0;
	medium[1] = 3.0;
	medium[2] = 4.0;
	LinearMotionND<double, 3> motion1(medium, low);
	LinearMotionND<double, 3> motion2(low, medium);
	LinearMotionND<double, 3> motion3(high, low);
	Interval<double> range(0.0, 1.0);

	// Output
	std::ostringstream linearMotionOutput;
	linearMotionOutput << motion1;
	EXPECT_EQ("([2,3,4] -> [1,2,3])", linearMotionOutput.str());

	// Triple Product a * (b X c)
	EXPECT_TRUE(tripleProduct(motion1, motion2, motion3, range).isApprox(Interval<double>(0.0, 0.0), epsilon));
}

}; // namespace Math
}; // namespace SurgSim

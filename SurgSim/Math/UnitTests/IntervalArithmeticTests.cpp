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

#include "SurgSim/Math/IntervalArithmetic.h"

namespace SurgSim
{

namespace Math
{

class IntervalArithmeticTests : public ::testing::Test
{
public:
	typedef Interval_nD<double, 2> IntervalDouble2;
	typedef Interval_nD<double, 3> IntervalDouble3;
};

TEST_F(IntervalArithmeticTests, IntervalInitializationTests)
{
	// Constructor tests
	EXPECT_NO_THROW(Interval<double> a);
	EXPECT_THROW(Interval<double> a(3.8, 3.7), SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(Interval<double> a(3.7, 3.8));
	Interval<double> b(9.8, 100);
	EXPECT_NO_THROW(Interval<double> a(b));
	Interval<double> a(b);
	EXPECT_TRUE(a == b);

	// Assignment and equal/not equal tests
	Interval<double> c;
	EXPECT_TRUE(c != b);
	EXPECT_FALSE(c == b);
	c = a;
	EXPECT_FALSE(c != b);
	EXPECT_TRUE(c == b);

	// Reordering initializers
	EXPECT_TRUE(b == SurgSim::Math::Interval<double>::minToMax(100, 9.8));
	EXPECT_TRUE(b == SurgSim::Math::Interval<double>::minToMax(12, 100, 9.8));
	EXPECT_TRUE(b == SurgSim::Math::Interval<double>::minToMax(12, 100, 9.8, 99));

	// Get upper and lower
	EXPECT_EQ(-5.0, Interval<double> (-5.0, 37).getMin());
	EXPECT_EQ(37.0, Interval<double> (-5.0, 37).getMax());
};

TEST_F(IntervalArithmeticTests, RangeTests)
{
	Interval<double> middle(10.0, 20.0);
	Interval<double> low(5.0, 9.999999999);
	Interval<double> high(20.00000001, 40.0);
	Interval<double> lowOverlap(5.0, 10.0);
	Interval<double> highOverlap(20.0, 40.0);
	Interval<double> contained(12.0, 15.0);
	Interval<double> contains(9.0, 21.0);

	// Overlap
	EXPECT_FALSE(middle.overlapsWith(low));
	EXPECT_FALSE(middle.overlapsWith(high));
	EXPECT_TRUE(middle.overlapsWith(lowOverlap));
	EXPECT_TRUE(middle.overlapsWith(highOverlap));
	EXPECT_TRUE(middle.overlapsWith(contained));
	EXPECT_TRUE(middle.overlapsWith(contains));

	// Contains value ...
	EXPECT_FALSE(middle.contains(9.999999999));
	EXPECT_FALSE(middle.contains(20.00000001));
	EXPECT_TRUE(middle.contains(10.0));
	EXPECT_TRUE(middle.contains(20.0));
	EXPECT_TRUE(middle.contains(15.0));

	Interval<double> aroundZero(-5.0, 13.0);
	Interval<double> zeroLow(-5.0, 0.0);
	Interval<double> zeroHigh(0.0, 13.0);
	Interval<double> almostZeroLow(-5.0, -0.000000001);
	Interval<double> almostZeroHigh(0.00000001, 13.0);

	// Contains zero ...
	EXPECT_FALSE(middle.containsZero());
	EXPECT_FALSE(almostZeroLow.containsZero());
	EXPECT_FALSE(almostZeroHigh.containsZero());
	EXPECT_TRUE(aroundZero.containsZero());
	EXPECT_TRUE(zeroLow.containsZero());
	EXPECT_TRUE(zeroHigh.containsZero());

	// Subrange tests
	EXPECT_EQ(Interval<double> (10.0, 15.0), middle.lowerHalf());
	EXPECT_EQ(Interval<double> (15.0, 20.0), middle.upperHalf());
};

TEST_F(IntervalArithmeticTests, ExtendRangeTests)
{
	Interval<double> middle(10.0, 20.0);
	Interval<double> original(middle);
	EXPECT_TRUE(original == middle);

	// Add onto both ends
	EXPECT_FALSE(middle.contains(8.0));
	EXPECT_FALSE(middle.contains(22.0));
	Interval<double> add(middle.addThickness(2.0));
	EXPECT_TRUE(add == middle);
	EXPECT_FALSE(original == middle);
	EXPECT_TRUE(middle.contains(8.0));
	EXPECT_TRUE(middle.contains(22.0));
	EXPECT_FALSE(middle.contains(7.999999999));
	EXPECT_FALSE(middle.contains(22.00000001));

	// Extend the interval ...
	original = middle;
	Interval<double> extend(middle.extendToInclude(12.0));
	EXPECT_TRUE(original == middle);
	EXPECT_TRUE(extend == middle);
	EXPECT_TRUE(middle.contains(8.0));
	EXPECT_TRUE(middle.contains(22.0));
	EXPECT_FALSE(middle.contains(7.999999999));
	EXPECT_FALSE(middle.contains(22.00000001));

	extend = middle.extendToInclude(5.0);
	EXPECT_FALSE(original == middle);
	EXPECT_TRUE(extend == middle);
	EXPECT_TRUE(middle == Interval<double>(5.0, 22));

	extend = middle.extendToInclude(24.0);
	EXPECT_TRUE(extend == middle);
	EXPECT_TRUE(middle == Interval<double>(5.0, 24.0));

	original = middle;
	extend = middle.extendToInclude(Interval<double> (5.0, 24.0));
	EXPECT_TRUE(extend == middle);
	EXPECT_TRUE(middle == original);

	extend = middle.extendToInclude(Interval<double> (4.0, 25.0));
	EXPECT_TRUE(extend == middle);
	EXPECT_FALSE(middle == original);
	EXPECT_TRUE(middle == Interval<double>(4.0, 25.0));

	extend = middle.extendToInclude(Interval<double> (3.0, 24.0));
	EXPECT_TRUE(extend == middle);
	EXPECT_TRUE(middle == Interval<double>(3.0, 25.0));

	extend = middle.extendToInclude(Interval<double> (7.0, 32.0));
	EXPECT_TRUE(extend == middle);
	EXPECT_TRUE(middle == Interval<double>(3.0, 32.0));
};

TEST_F(IntervalArithmeticTests, OperatorTests)
{
	Interval<double> initial(10.0, 20.0);

	// +
	EXPECT_EQ(Interval<double>(12, 30), initial + Interval<double>(2.0, 10.0));
	EXPECT_EQ(Interval<double>(20, 30), initial + 10);

	// +=
	Interval<double> dummy(initial);
	dummy += Interval<double>(2.0, 10.0);
	EXPECT_EQ(Interval<double>(12, 30), dummy);
	dummy = initial;
	dummy += 10.0;
	EXPECT_EQ(Interval<double>(20, 30), dummy);

	// -
	EXPECT_EQ(Interval<double>(0.0, 18.0), initial - Interval<double>(2.0, 10.0));
	EXPECT_EQ(Interval<double>(0, 10), initial - 10);

	// -=
	dummy = initial;
	dummy -= Interval<double>(2.0, 10.0);
	EXPECT_EQ(Interval<double>(0.0, 18.0), dummy);
	dummy = initial;
	dummy -= 10.0;
	EXPECT_EQ(Interval<double>(0.0, 10.0), dummy);

	// Negation
	EXPECT_EQ(Interval<double>(-20.0, -10.0), -initial);

	// *
	EXPECT_EQ(Interval<double>(20.0, 200.0), initial * Interval<double>(2.0, 10.0));
	EXPECT_EQ(Interval<double>(-200.0, 40.0), -initial * Interval<double>(-2.0, 10.0));
	EXPECT_EQ(Interval<double>(100.0, 200.0), initial * 10);
	EXPECT_EQ(Interval<double>(100.0, 200.0), -initial * -10);

	// *=
	dummy = initial;
	dummy *= Interval<double>(2.0, 10.0);
	EXPECT_EQ(Interval<double>(20.0, 200.0), dummy);
	dummy = -initial;
	dummy *= Interval<double>(-2.0, 10.0);
	EXPECT_EQ(Interval<double>(-200.0, 40.0), dummy);
	dummy = initial;
	dummy *= 10.0;
	EXPECT_EQ(Interval<double>(100.0, 200.0), initial * 10);
	dummy = -initial;
	dummy *= -10.0;
	EXPECT_EQ(Interval<double>(100.0, 200.0), -initial * -10);

	// /
	Interval<double> zeroInterval(-5.0, 5.0);
	EXPECT_THROW(initial / zeroInterval, SurgSim::Framework::AssertionFailure);
	EXPECT_EQ(Interval<double>(20.0, 200.0), initial / Interval<double>(0.1, 0.5));

	// /=
	dummy = zeroInterval;
	EXPECT_THROW(dummy /= zeroInterval, SurgSim::Framework::AssertionFailure);
	dummy = initial;
	dummy /= Interval<double>(0.1, 0.5);
	EXPECT_EQ(Interval<double>(20.0, 200.0), dummy);

	// Inverse
	EXPECT_THROW(zeroInterval.inverse(), SurgSim::Framework::AssertionFailure);
	EXPECT_EQ(Interval<double> (2, 10), Interval<double>(0.1, 0.5).inverse());

	// Square
	EXPECT_EQ(Interval<double> (0, 25), zeroInterval.square());
	EXPECT_EQ(Interval<double> (0, 25), (-zeroInterval).square());
	EXPECT_EQ(Interval<double> (100, 400), initial.square());
	EXPECT_EQ(Interval<double> (100, 400), (-initial).square());
};

// Interval nD tests
TEST_F(IntervalArithmeticTests, IntervalnDInitializationTests)
{
	// Constructor tests
	Interval<double> testInterval(3.7, 3.8);
	double minimums[2] = {1.1, 4.4};
	double maximums[2] = {4.1, 3.2};

	Interval<double> testIntervalArray[2];
	testIntervalArray[0] = Interval<double> (1.0, 2.0);
	testIntervalArray[1] = Interval<double> (2.0, 3.0);

	EXPECT_NO_THROW(IntervalDouble2 a);
	EXPECT_NO_THROW(IntervalDouble2 a(testIntervalArray));
	EXPECT_NO_THROW(IntervalDouble2 a(minimums, maximums));
	EXPECT_NO_THROW(IntervalDouble2 a(IntervalDouble2(minimums, maximums)));

	IntervalDouble2 b(minimums, maximums);
	IntervalDouble2 a(b);
	EXPECT_TRUE(a == b);

	// Assignment and equal/not equal tests
	IntervalDouble2 c;
	EXPECT_TRUE(c != b);
	EXPECT_FALSE(c == b);
	c = a;
	EXPECT_FALSE(c != b);
	EXPECT_TRUE(c == b);

	// Getting components and the reordering initializer
	Interval<double> axis0(b.getAxis(0));
	Interval<double> axis1(b.getAxis(1));

	EXPECT_TRUE(axis0 == Interval<double>(1.1, 4.1));
	EXPECT_TRUE(axis1 == Interval<double>(3.2, 4.4));
};

TEST_F(IntervalArithmeticTests, IntervalnDRangeTests)
{
	double minimumsShape[2] = {1.1, 3.2};
	double maximumsShape[2] = {4.1, 4.4};

	double noOverlapMin[2] = {1.1, 4.5};
	double noOverlapMax[2] = {4.5, 4.8};

	double overlapMin[2] = {1.7, 2.0};
	double overlapMax[2] = {4.3, 4.7};

	IntervalDouble2 shape(minimumsShape, maximumsShape);

	EXPECT_FALSE(shape.overlapsWith(IntervalDouble2(noOverlapMin, noOverlapMax)));
	EXPECT_TRUE(shape.overlapsWith(IntervalDouble2(overlapMin, overlapMax)));
};

TEST_F(IntervalArithmeticTests, IntervalnDExtendRangeTests)
{
	double minimumsShape[2] = {1.1, 3.2};
	double maximumsShape[2] = {4.1, 4.4};

	double finalMin[2] = { -5.9, -3.8};
	double finalMax[2] = {11.1, 11.4};

	IntervalDouble2 shape(minimumsShape, maximumsShape);
	shape.addThickness(7.0);
	EXPECT_EQ(IntervalDouble2(finalMin, finalMax), shape);
};

TEST_F(IntervalArithmeticTests, IntervalnDOperatorTests)
{
	double minimumsShape[2] = {1.1, 3.2};
	double maximumsShape[2] = {4.1, 4.4};

	double deltaMin[2] = { -1.0, -2.0};
	double deltaMax[2] = {3.0, 4.0};

	double nonZeroDeltaMin[2] = { -3.0, 2.0};
	double nonZeroDeltaMax[2] = { -1.0, 4.0};

	IntervalDouble2 initial(minimumsShape, maximumsShape);
	IntervalDouble2 delta(deltaMin, deltaMax);
	IntervalDouble2 nonZeroDelta(nonZeroDeltaMin, nonZeroDeltaMax);
	IntervalDouble2 dummy;

	// +
	EXPECT_EQ((Interval<double>(1.1, 4.1) + Interval<double>(-1.0, 3.0)), (initial + delta).getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) + Interval<double>(-2.0, 4.0)), (initial + delta).getAxis(1));

	// +=
	dummy = initial;
	dummy += delta;
	EXPECT_EQ((Interval<double>(1.1, 4.1) + Interval<double>(-1.0, 3.0)), dummy.getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) + Interval<double>(-2.0, 4.0)), dummy.getAxis(1));

	// -
	EXPECT_EQ((Interval<double>(1.1, 4.1) - Interval<double>(-1.0, 3.0)), (initial - delta).getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) - Interval<double>(-2.0, 4.0)), (initial - delta).getAxis(1));

	// -=
	dummy = initial;
	dummy -= delta;
	EXPECT_EQ((Interval<double>(1.1, 4.1) - Interval<double>(-1.0, 3.0)), dummy.getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) - Interval<double>(-2.0, 4.0)), dummy.getAxis(1));

	// *
	EXPECT_EQ((Interval<double>(1.1, 4.1) * Interval<double>(-1.0, 3.0)), (initial * delta).getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) * Interval<double>(-2.0, 4.0)), (initial * delta).getAxis(1));

	// *=
	dummy = initial;
	dummy *= delta;
	EXPECT_EQ((Interval<double>(1.1, 4.1) * Interval<double>(-1.0, 3.0)), dummy.getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) * Interval<double>(-2.0, 4.0)), dummy.getAxis(1));

	// /
	EXPECT_EQ((Interval<double>(1.1, 4.1) / Interval<double>(-3.0, -1.0)), (initial / nonZeroDelta).getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) / Interval<double>(2.0, 4.0)), (initial / nonZeroDelta).getAxis(1));

	// /=
	dummy = initial;
	dummy /= nonZeroDelta;
	EXPECT_EQ((Interval<double>(1.1, 4.1) / Interval<double>(-3.0, -1.0)), dummy.getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) / Interval<double>(2.0, 4.0)), dummy.getAxis(1));

	// Inverse
	EXPECT_EQ(Interval<double>(1.1, 4.1).inverse(), initial.inverse().getAxis(0));
	EXPECT_EQ(Interval<double>(3.2, 4.4).inverse(), initial.inverse().getAxis(1));

	// Dot product
	Interval<double> dotValue(initial.dotProduct(delta));
	EXPECT_EQ((initial.getAxis(0) * delta.getAxis(0)) + (initial.getAxis(1) * delta.getAxis(1)), dotValue);

	// Magnitude and magnitude squared
	Interval<double> magnitudeSquaredValue(initial.magnitudeSquared());
	EXPECT_EQ((initial.getAxis(0).square() + initial.getAxis(1).square()), magnitudeSquaredValue);

	Interval<double> magnitudeValue(initial.magnitude());
	EXPECT_EQ(std::sqrt(magnitudeSquaredValue.getMin()), magnitudeValue.getMin());
	EXPECT_EQ(std::sqrt(magnitudeSquaredValue.getMax()), magnitudeValue.getMax());
};

// Interval nD tests
TEST_F(IntervalArithmeticTests, Interval3DInitializationTests)
{
	// Constructor tests
	Interval<double> testInterval(3.7, 3.8);
	double minimums[3] = {1.1, 4.4, -7.2};
	double maximums[3] = {4.1, 3.2, -1.0};

	Interval<double> testIntervalArray[3];
	testIntervalArray[0] = Interval<double> (1.0, 2.0);
	testIntervalArray[1] = Interval<double> (2.0, 3.0);
	testIntervalArray[2] = Interval<double> (3.0, 4.0);

	EXPECT_NO_THROW(IntervalDouble3 a);
	EXPECT_NO_THROW(IntervalDouble3 a(testIntervalArray));
	EXPECT_NO_THROW(IntervalDouble3 a(Interval<double> (1.0, 2.0),
									  Interval<double> (2.0, 3.0),
									  Interval<double> (3.0, 4.0)));
	EXPECT_NO_THROW(IntervalDouble3 a(minimums, maximums));
	EXPECT_NO_THROW(IntervalDouble3 a(IntervalDouble3(minimums, maximums)));

	IntervalDouble3 b(minimums, maximums);
	IntervalDouble3 a(b);
	EXPECT_TRUE(a == b);

// Assignment and equal/not equal tests
	IntervalDouble3 c;
	EXPECT_TRUE(c != b);
	EXPECT_FALSE(c == b);
	c = a;
	EXPECT_FALSE(c != b);
	EXPECT_TRUE(c == b);

// Getting components and the reordering initializer
	Interval<double> axis0 = b.getAxis(0);
	Interval<double> axis1 = b.getAxis(1);
	Interval<double> axis2 = b.getAxis(2);

	EXPECT_TRUE(axis0 == Interval<double>(1.1, 4.1));
	EXPECT_TRUE(axis1 == Interval<double>(3.2, 4.4));
	EXPECT_TRUE(axis2 == Interval<double>(-7.2, -1.0));
};

TEST_F(IntervalArithmeticTests, Interval3DRangeTests)
{
	double minimumsShape[3] = {1.1, 3.2, -7.2};
	double maximumsShape[3] = {4.1, 4.4, -1.0};

	double noOverlapMin[3] = {1.1, 4.5, -8.0};
	double noOverlapMax[3] = {4.5, 4.8, -3.0};

	double overlapMin[3] = {1.7, 2.0, -8.0};
	double overlapMax[3] = {4.3, 4.7, -3.0};

	IntervalDouble3 shape(minimumsShape, maximumsShape);

	EXPECT_FALSE(shape.overlapsWith(IntervalDouble3(noOverlapMin, noOverlapMax)));
	EXPECT_TRUE(shape.overlapsWith(IntervalDouble3(overlapMin, overlapMax)));
};

TEST_F(IntervalArithmeticTests, Interval3DExtendRangeTests)
{
	double minimumsShape[3] = {1.1, 3.2, -7.2};
	double maximumsShape[3] = {4.1, 4.4, -1.0};

	double finalMin[3] = { -5.9, -3.8, -14.2};
	double finalMax[3] = {11.1, 11.4, 6.0};

	IntervalDouble3 shape(minimumsShape, maximumsShape);
	shape.addThickness(7.0);
	EXPECT_EQ(IntervalDouble3(finalMin, finalMax), shape);
};

TEST_F(IntervalArithmeticTests, Interval3DOperatorTests)
{
	double minimumsShape[3] = {1.1, 3.2, -7.2};
	double maximumsShape[3] = {4.1, 4.4, -1.0};

	double deltaMin[3] = { -1.0, -2.0, -3.0};
	double deltaMax[3] = {3.0, 4.0, 5.0};

	double nonZeroDeltaMin[3] = { -3.0, 2.0, 3.0};
	double nonZeroDeltaMax[3] = { -1.0, 4.0, 5.0};

	IntervalDouble3 initial(minimumsShape, maximumsShape);
	IntervalDouble3 delta(deltaMin, deltaMax);
	IntervalDouble3 nonZeroDelta(nonZeroDeltaMin, nonZeroDeltaMax);
	IntervalDouble3 dummy;

	// +
	EXPECT_EQ((Interval<double>(1.1, 4.1) + Interval<double>(-1.0, 3.0)), (initial + delta).getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) + Interval<double>(-2.0, 4.0)), (initial + delta).getAxis(1));
	EXPECT_EQ((Interval<double>(-7.2, -1.0) + Interval<double>(-3.0, 5.0)), (initial + delta).getAxis(2));

	// +=
	dummy = initial;
	dummy += delta;
	EXPECT_EQ((Interval<double>(1.1, 4.1) + Interval<double>(-1.0, 3.0)), dummy.getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) + Interval<double>(-2.0, 4.0)), dummy.getAxis(1));
	EXPECT_EQ((Interval<double>(-7.2, -1.0) + Interval<double>(-3.0, 5.0)), dummy.getAxis(2));

	// -
	EXPECT_EQ((Interval<double>(1.1, 4.1) - Interval<double>(-1.0, 3.0)), (initial - delta).getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) - Interval<double>(-2.0, 4.0)), (initial - delta).getAxis(1));
	EXPECT_EQ((Interval<double>(-7.2, -1.0) - Interval<double>(-3.0, 5.0)), (initial - delta).getAxis(2));

	// -=
	dummy = initial;
	dummy -= delta;
	EXPECT_EQ((Interval<double>(1.1, 4.1) - Interval<double>(-1.0, 3.0)), dummy.getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) - Interval<double>(-2.0, 4.0)), dummy.getAxis(1));
	EXPECT_EQ((Interval<double>(-7.2, -1.0) - Interval<double>(-3.0, 5.0)), dummy.getAxis(2));

	// *
	EXPECT_EQ((Interval<double>(1.1, 4.1) * Interval<double>(-1.0, 3.0)), (initial * delta).getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) * Interval<double>(-2.0, 4.0)), (initial * delta).getAxis(1));
	EXPECT_EQ((Interval<double>(-7.2, -1.0) * Interval<double>(-3.0, 5.0)), (initial * delta).getAxis(2));

	// *=
	dummy = initial;
	dummy *= delta;
	EXPECT_EQ((Interval<double>(1.1, 4.1) * Interval<double>(-1.0, 3.0)), dummy.getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) * Interval<double>(-2.0, 4.0)), dummy.getAxis(1));
	EXPECT_EQ((Interval<double>(-7.2, -1.0) * Interval<double>(-3.0, 5.0)), dummy.getAxis(2));

	// /
	EXPECT_EQ((Interval<double>(1.1, 4.1) / Interval<double>(-3.0, -1.0)), (initial / nonZeroDelta).getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) / Interval<double>(2.0, 4.0)), (initial / nonZeroDelta).getAxis(1));
	EXPECT_EQ((Interval<double>(-7.2, -1.0) / Interval<double>(3.0, 5.0)), (initial / nonZeroDelta).getAxis(2));

	// /=
	dummy = initial;
	dummy /= nonZeroDelta;
	EXPECT_EQ((Interval<double>(1.1, 4.1) / Interval<double>(-3.0, -1.0)), dummy.getAxis(0));
	EXPECT_EQ((Interval<double>(3.2, 4.4) / Interval<double>(2.0, 4.0)), dummy.getAxis(1));
	EXPECT_EQ((Interval<double>(-7.2, -1.0) / Interval<double>(3.0, 5.0)), dummy.getAxis(2));

	// Inverse
	EXPECT_EQ(Interval<double>(1.1, 4.1).inverse(), initial.inverse().getAxis(0));
	EXPECT_EQ(Interval<double>(3.2, 4.4).inverse(), initial.inverse().getAxis(1));
	EXPECT_EQ(Interval<double>(-7.2, -1.0).inverse(), initial.inverse().getAxis(2));

	// Dot product
	Interval<double> dotValue = initial.dotProduct(delta);
	EXPECT_EQ((initial.getAxis(0) * delta.getAxis(0)) +
			  (initial.getAxis(1) * delta.getAxis(1)) +
			  (initial.getAxis(2) * delta.getAxis(2)), dotValue);

	// Cross product
	IntervalDouble3 crossValue = initial.crossProduct(delta);
	EXPECT_EQ((initial.getAxis(1) * delta.getAxis(2) - initial.getAxis(2) * delta.getAxis(1)), crossValue.getAxis(0));
	EXPECT_EQ((initial.getAxis(2) * delta.getAxis(0) - initial.getAxis(0) * delta.getAxis(2)), crossValue.getAxis(1));
	EXPECT_EQ((initial.getAxis(0) * delta.getAxis(1) - initial.getAxis(1) * delta.getAxis(0)), crossValue.getAxis(2));

	// Magnitude and magnitude squared
	Interval<double> magnitudeSquaredValue = initial.magnitudeSquared();
	EXPECT_EQ((initial.getAxis(0).square() + initial.getAxis(1).square() + initial.getAxis(2).square()),
			  magnitudeSquaredValue);

	Interval<double> magnitudeValue = initial.magnitude();
	EXPECT_EQ(std::sqrt(magnitudeSquaredValue.getMin()), magnitudeValue.getMin());
	EXPECT_EQ(std::sqrt(magnitudeSquaredValue.getMax()), magnitudeValue.getMax());
};

TEST_F(IntervalArithmeticTests, IntervalUtilityTests)
{
	Interval<double> initial(10.0, 20.0);
	Interval<double> delta(3.0, 4.0);
	Interval<double> resultLoad(-2.0, -1.0);
	Interval<double> result;

	// Constant + interval
	EXPECT_EQ(initial + 11.5, 11.5 + initial);

	// Constant * interval
	EXPECT_EQ(initial * 11.5, 11.5 * initial);

	// Output
	std::ostringstream intervalOutput;
	intervalOutput << initial;
	EXPECT_TRUE(intervalOutput.str() == "[10,20]");

	// +
	IntervalArithmetic_add(initial, delta, result);
	EXPECT_EQ(initial + delta, result);

	// +=( + )
	result = resultLoad;
	IntervalArithmetic_addadd(initial, delta, result);
	EXPECT_EQ(resultLoad + initial + delta, result);

	// -
	IntervalArithmetic_sub(initial, delta, result);
	EXPECT_EQ(initial - delta, result);

	// +=( - )
	result = resultLoad;
	IntervalArithmetic_addsub(initial, delta, result);
	EXPECT_EQ(resultLoad + (initial - delta), result);

	// *
	IntervalArithmetic_mul(initial, delta, result);
	EXPECT_EQ(initial * delta, result);

	// += ( * )
	result = resultLoad;
	IntervalArithmetic_addmul(initial, delta, result);
	EXPECT_EQ(resultLoad + (initial * delta), result);

	// -= ( * )
	result = resultLoad;
	IntervalArithmetic_submul(initial, delta, result);
	EXPECT_EQ(resultLoad - (initial * delta), result);
};

TEST_F(IntervalArithmeticTests, IntervalnDUtilityTests)
{
	double minimumsShape[2] = {1.1, 3.2};
	double maximumsShape[2] = {4.1, 4.4};

	IntervalDouble2 shape(minimumsShape, maximumsShape);

	// Output
	std::ostringstream intervalOutput;
	intervalOutput << shape;
	EXPECT_TRUE(intervalOutput.str() == "([1.1,4.1];[3.2,4.4])");
};

TEST_F(IntervalArithmeticTests, Interval3DUtilityTests)
{
	double minimumsShape[3] = {1.1, 3.2, -7.2};
	double maximumsShape[3] = {4.1, 4.4, -1.0};

	double deltaMin[3] = { -1.0, -2.0, -3.0};
	double deltaMax[3] = {3.0, 4.0, 5.0};

	IntervalDouble3 initial(minimumsShape, maximumsShape);
	IntervalDouble3 delta(deltaMin, deltaMax);
	IntervalDouble3 result;
	Interval<double> intervalResult;

	// +
	IntervalArithmetic_add(initial, delta, result);
	EXPECT_EQ(initial + delta, result);

	// -
	IntervalArithmetic_sub(initial, delta, result);
	EXPECT_EQ(initial - delta, result);

	// dot product
	IntervalArithmetic_dotProduct(initial, delta, intervalResult);
	EXPECT_EQ(initial.dotProduct(delta), intervalResult);

	// Cross product
	IntervalArithmetic_crossProduct(initial, delta, result);
	EXPECT_EQ(initial.crossProduct(delta), result);
};

}; // namespace Math

}; // namespace SurgSim

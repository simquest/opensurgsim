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
/// Tests for the IntervalArithmetic functions.

#include <array>

#include <gtest/gtest.h>

#include "SurgSim/Math/IntervalArithmetic.h"

namespace SurgSim
{

namespace Math
{

namespace
{
double epsilon = 1.0e-10;
double delta = 1.0e-11;
}

class IntervalArithmeticTests : public ::testing::Test
{
public:
	typedef IntervalND<double, 2> IntervalDouble2;
	typedef IntervalND<double, 3> IntervalDouble3;

	Interval<double> testIntervalMoveConstructor(Interval<double> dummy)
	{
		Interval<double> ret;
		ret = dummy;
		return ret;
	}

	IntervalND<double, 2> testIntervalNDMoveConstructor(IntervalND<double, 2> dummy)
	{
		IntervalND<double, 2> ret;
		ret = dummy;
		return ret;
	}

	IntervalND<double, 3> testInterval3DMoveConstructor(IntervalND<double, 3> dummy)
	{
		IntervalND<double, 3> ret;
		ret = dummy;
		return ret;
	}
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
	EXPECT_NO_THROW(testIntervalMoveConstructor(b));
	Interval<double> f = testIntervalMoveConstructor(b);
	EXPECT_TRUE(f == b);

	// Assignment and equal/not equal tests
	Interval<double> c;
	EXPECT_TRUE(c != b);
	EXPECT_FALSE(c == b);
	c = a;
	EXPECT_FALSE(c != b);
	EXPECT_TRUE(c == b);
	Interval<double> e;
	e = (b * 1);
	EXPECT_TRUE(e == b);
	Interval<double> ePlusEpsilon = e + delta;
	Interval<double> ePlus2Epsilon = e + (2 * epsilon);
	EXPECT_TRUE(e.isApprox(ePlusEpsilon, epsilon));
	EXPECT_TRUE(ePlusEpsilon.isApprox(e, epsilon));
	EXPECT_FALSE(e.isApprox(ePlus2Epsilon, epsilon));
	EXPECT_FALSE(ePlus2Epsilon.isApprox(e, epsilon));

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
	Interval<double> low(5.0, 10 - epsilon);
	Interval<double> high(20 + epsilon, 40.0);
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
	EXPECT_FALSE(middle.contains(10 - epsilon));
	EXPECT_FALSE(middle.contains(20 + epsilon));
	EXPECT_TRUE(middle.contains(10.0));
	EXPECT_TRUE(middle.contains(20.0));
	EXPECT_TRUE(middle.contains(15.0));

	Interval<double> aroundZero(-5.0, 13.0);
	Interval<double> zeroLow(-5.0, 0.0);
	Interval<double> zeroHigh(0.0, 13.0);
	Interval<double> almostZeroLow(-5.0, -epsilon);
	Interval<double> almostZeroHigh(epsilon, 13.0);

	// Contains zero ...
	EXPECT_FALSE(middle.containsZero());
	EXPECT_FALSE(almostZeroLow.containsZero());
	EXPECT_FALSE(almostZeroHigh.containsZero());
	EXPECT_TRUE(aroundZero.containsZero());
	EXPECT_TRUE(zeroLow.containsZero());
	EXPECT_TRUE(zeroHigh.containsZero());

	// Subrange tests
	EXPECT_TRUE(middle.lowerHalf().isApprox(Interval<double> (10.0, 15.0), epsilon));
	EXPECT_TRUE(middle.upperHalf().isApprox(Interval<double> (15.0, 20.0), epsilon));
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
	EXPECT_TRUE(Interval<double>(12, 30).isApprox((initial + Interval<double>(2.0, 10.0)), epsilon));
	EXPECT_TRUE(Interval<double>(20, 30).isApprox((initial + 10), epsilon));

	// +=
	Interval<double> dummy(initial);
	dummy += Interval<double>(2.0, 10.0);
	EXPECT_TRUE(dummy.isApprox(Interval<double>(12, 30), epsilon));
	dummy = initial;
	dummy += 10.0;
	EXPECT_TRUE(dummy.isApprox(Interval<double>(20, 30), epsilon));

	// -
	EXPECT_TRUE(Interval<double>(0.0, 18.0).isApprox((initial - Interval<double>(2.0, 10.0)), epsilon));
	EXPECT_TRUE(Interval<double>(0, 10).isApprox((initial - 10), epsilon));

	// -=
	dummy = initial;
	dummy -= Interval<double>(2.0, 10.0);
	EXPECT_TRUE(Interval<double>(0.0, 18.0).isApprox(dummy, epsilon));
	dummy = initial;
	dummy -= 10.0;
	EXPECT_TRUE(Interval<double>(0.0, 10.0).isApprox(dummy, epsilon));

	// Negation
	EXPECT_TRUE(Interval<double>(-20.0, -10.0).isApprox(-initial, epsilon));

	// *
	EXPECT_TRUE(Interval<double>(20.0, 200.0).isApprox((initial * Interval<double>(2.0, 10.0)), epsilon));
	EXPECT_TRUE(Interval<double>(-200.0, 40.0).isApprox((-initial * Interval<double>(-2.0, 10.0)), epsilon));
	EXPECT_TRUE(Interval<double>(100.0, 200.0).isApprox((initial * 10), epsilon));
	EXPECT_TRUE(Interval<double>(100.0, 200.0).isApprox((-initial * -10), epsilon));

	// *=
	dummy = initial;
	dummy *= Interval<double>(2.0, 10.0);
	EXPECT_TRUE(Interval<double>(20.0, 200.0).isApprox(dummy, epsilon));
	dummy = -initial;
	dummy *= Interval<double>(-2.0, 10.0);
	EXPECT_TRUE(Interval<double>(-200.0, 40.0).isApprox(dummy, epsilon));
	dummy = initial;
	dummy *= 10.0;
	EXPECT_TRUE(Interval<double>(100.0, 200.0).isApprox((initial * 10), epsilon));
	dummy = -initial;
	dummy *= -10.0;
	EXPECT_TRUE(Interval<double>(100.0, 200.0).isApprox((-initial * -10), epsilon));

	// /
	Interval<double> zeroInterval(-5.0, 5.0);
	EXPECT_THROW(initial / zeroInterval, SurgSim::Framework::AssertionFailure);
	EXPECT_TRUE(Interval<double>(20.0, 200.0).isApprox((initial / Interval<double>(0.1, 0.5)), epsilon));

	// /=
	dummy = zeroInterval;
	EXPECT_THROW(dummy /= zeroInterval, SurgSim::Framework::AssertionFailure);
	dummy = initial;
	dummy /= Interval<double>(0.1, 0.5);
	EXPECT_TRUE(Interval<double>(20.0, 200.0).isApprox(dummy, epsilon));

	// Inverse
	EXPECT_THROW(zeroInterval.inverse(), SurgSim::Framework::AssertionFailure);
	EXPECT_TRUE(Interval<double> (2, 10).isApprox(Interval<double>(0.1, 0.5).inverse(), epsilon));

	// Square
	EXPECT_TRUE(Interval<double> (0, 25).isApprox(zeroInterval.square(), epsilon));
	EXPECT_TRUE(Interval<double> (0, 25).isApprox((-zeroInterval).square(), epsilon));
	EXPECT_TRUE(Interval<double> (100, 400).isApprox(initial.square(), epsilon));
	EXPECT_TRUE(Interval<double> (100, 400).isApprox((-initial).square(), epsilon));
};

// Interval ND tests
TEST_F(IntervalArithmeticTests, IntervalNDInitializationTests)
{
	// Constructor tests
	Interval<double> testInterval(3.7, 3.8);
	std::array<double, 2> minimums = {1.1, 4.4};
	std::array<double, 2> maximums = {4.1, 3.2};
	std::array<double, 2> minimumsPlusEpsilon = {1.1 + delta, 4.4 + delta};
	std::array<double, 2> maximumsPlusEpsilon = {4.1 + delta, 3.2 + delta};
	std::array<double, 2> maximumsPlus2Epsilon = {4.1 + (2 * epsilon), 3.2 + delta};

	std::array<Interval<double>, 2> testIntervalArray;
	testIntervalArray[0] = Interval<double> (1.0, 2.0);
	testIntervalArray[1] = Interval<double> (2.0, 3.0);

	EXPECT_NO_THROW(IntervalDouble2 a);
	EXPECT_NO_THROW(IntervalDouble2 a(testIntervalArray));
	EXPECT_NO_THROW(IntervalDouble2 a(minimums, maximums));
	EXPECT_NO_THROW(IntervalDouble2 a(IntervalDouble2(minimums, maximums)));

	IntervalDouble2 b(minimums, maximums);
	IntervalDouble2 a(b);
	EXPECT_TRUE(a == b);
	EXPECT_NO_THROW(testIntervalNDMoveConstructor(b));
	IntervalDouble2 f = testIntervalNDMoveConstructor(b);
	EXPECT_TRUE(f == b);

	// Assignment and equal/not equal tests
	IntervalDouble2 c;
	EXPECT_TRUE(c != b);
	EXPECT_FALSE(c == b);
	c = a;
	EXPECT_FALSE(c != b);
	EXPECT_TRUE(c == b);
	IntervalDouble2 e;
	e = (b * c);
	EXPECT_TRUE(e == (b * c));

	IntervalDouble2 bPlusEpsilon(minimumsPlusEpsilon, maximumsPlusEpsilon);
	IntervalDouble2 bPlus2Epsilon(minimumsPlusEpsilon, maximumsPlus2Epsilon);
	EXPECT_TRUE(b.isApprox(bPlusEpsilon, epsilon));
	EXPECT_TRUE(bPlusEpsilon.isApprox(b, epsilon));
	EXPECT_FALSE(b.isApprox(bPlus2Epsilon, epsilon));
	EXPECT_FALSE(bPlus2Epsilon.isApprox(b, epsilon));

	// Getting components and the reordering initializer
	Interval<double> axis0(b.getAxis(0));
	Interval<double> axis1(b.getAxis(1));

	EXPECT_TRUE(axis0 == Interval<double>(1.1, 4.1));
	EXPECT_TRUE(axis1 == Interval<double>(3.2, 4.4));
};

TEST_F(IntervalArithmeticTests, IntervalNDRangeTests)
{
	std::array<double, 2> minimumsShape = {1.1, 3.2};
	std::array<double, 2> maximumsShape = {4.1, 4.4};

	std::array<double, 2> noOverlapMin = {1.1, 4.5};
	std::array<double, 2> noOverlapMax = {4.5, 4.8};

	std::array<double, 2> overlapMin = {1.7, 2.0};
	std::array<double, 2> overlapMax = {4.3, 4.7};

	IntervalDouble2 shape(minimumsShape, maximumsShape);

	EXPECT_FALSE(shape.overlapsWith(IntervalDouble2(noOverlapMin, noOverlapMax)));
	EXPECT_TRUE(shape.overlapsWith(IntervalDouble2(overlapMin, overlapMax)));
};

TEST_F(IntervalArithmeticTests, IntervalNDExtendRangeTests)
{
	std::array<double, 2> minimumsShape = {1.1, 3.2};
	std::array<double, 2> maximumsShape = {4.1, 4.4};

	std::array<double, 2> finalMin = { -5.9, -3.8};
	std::array<double, 2> finalMax = {11.1, 11.4};

	IntervalDouble2 shape(minimumsShape, maximumsShape);
	shape.addThickness(7.0);
	EXPECT_EQ(IntervalDouble2(finalMin, finalMax), shape);
};

TEST_F(IntervalArithmeticTests, IntervalNDOperatorTests)
{
	std::array<double, 2> minimumsShape = {1.1, 3.2};
	std::array<double, 2> maximumsShape = {4.1, 4.4};

	std::array<double, 2> deltaMin = { -1.0, -2.0};
	std::array<double, 2> deltaMax = {3.0, 4.0};

	std::array<double, 2> nonZeroDeltaMin = { -3.0, 2.0};
	std::array<double, 2> nonZeroDeltaMax = { -1.0, 4.0};

	IntervalDouble2 initial(minimumsShape, maximumsShape);
	IntervalDouble2 deltaInterval(deltaMin, deltaMax);
	IntervalDouble2 nonZeroDelta(nonZeroDeltaMin, nonZeroDeltaMax);
	IntervalDouble2 dummy;

	// +
	EXPECT_TRUE((Interval<double>(1.1, 4.1) + Interval<double>(-1.0, 3.0)).isApprox(
					(initial + deltaInterval).getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) + Interval<double>(-2.0, 4.0)).isApprox(
					(initial + deltaInterval).getAxis(1), epsilon));

	// +=
	dummy = initial;
	dummy += deltaInterval;
	EXPECT_TRUE((Interval<double>(1.1, 4.1) + Interval<double>(-1.0, 3.0)).isApprox(dummy.getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) + Interval<double>(-2.0, 4.0)).isApprox(dummy.getAxis(1), epsilon));

	// -
	EXPECT_TRUE((Interval<double>(1.1, 4.1) - Interval<double>(-1.0, 3.0)).isApprox(
					(initial - deltaInterval).getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) - Interval<double>(-2.0, 4.0)).isApprox(
					(initial - deltaInterval).getAxis(1), epsilon));

	// -=
	dummy = initial;
	dummy -= deltaInterval;
	EXPECT_TRUE((Interval<double>(1.1, 4.1) - Interval<double>(-1.0, 3.0)).isApprox(dummy.getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) - Interval<double>(-2.0, 4.0)).isApprox(dummy.getAxis(1), epsilon));

	// *
	EXPECT_TRUE((Interval<double>(1.1, 4.1) * Interval<double>(-1.0, 3.0)).isApprox(
					(initial * deltaInterval).getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) * Interval<double>(-2.0, 4.0)).isApprox(
					(initial * deltaInterval).getAxis(1), epsilon));

	// *=
	dummy = initial;
	dummy *= deltaInterval;
	EXPECT_TRUE((Interval<double>(1.1, 4.1) * Interval<double>(-1.0, 3.0)).isApprox(dummy.getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) * Interval<double>(-2.0, 4.0)).isApprox(dummy.getAxis(1), epsilon));

	// /
	EXPECT_TRUE((Interval<double>(1.1, 4.1) / Interval<double>(-3.0, -1.0)).isApprox(
					(initial / nonZeroDelta).getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) / Interval<double>(2.0, 4.0)).isApprox(
					(initial / nonZeroDelta).getAxis(1), epsilon));

	// /=
	dummy = initial;
	dummy /= nonZeroDelta;
	EXPECT_TRUE((Interval<double>(1.1, 4.1) / Interval<double>(-3.0, -1.0)).isApprox(dummy.getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) / Interval<double>(2.0, 4.0)).isApprox(dummy.getAxis(1), epsilon));

	// Inverse
	EXPECT_TRUE(Interval<double>(1.1, 4.1).inverse().isApprox(initial.inverse().getAxis(0), epsilon));
	EXPECT_TRUE(Interval<double>(3.2, 4.4).inverse().isApprox(initial.inverse().getAxis(1), epsilon));

	// Dot product
	Interval<double> dotValue(initial.dotProduct(deltaInterval));
	EXPECT_TRUE(dotValue.isApprox((initial.getAxis(0) * deltaInterval.getAxis(0)) +
								  (initial.getAxis(1) * deltaInterval.getAxis(1)), epsilon));

	// Magnitude and magnitude squared
	Interval<double> magnitudeSquaredValue(initial.magnitudeSquared());
	EXPECT_TRUE(magnitudeSquaredValue.isApprox((initial.getAxis(0).square() + initial.getAxis(1).square()), epsilon));

	Interval<double> magnitudeValue(initial.magnitude());
	EXPECT_NEAR(std::sqrt(magnitudeSquaredValue.getMin()), magnitudeValue.getMin(), epsilon);
	EXPECT_NEAR(std::sqrt(magnitudeSquaredValue.getMax()), magnitudeValue.getMax(), epsilon);
};

// Interval ND tests
TEST_F(IntervalArithmeticTests, Interval3DInitializationTests)
{
	// Constructor tests
	Interval<double> testInterval(3.7, 3.8);
	std::array<double, 3> minimums = {1.1, 4.4, -7.2};
	std::array<double, 3> maximums = {4.1, 3.2, -1.0};
	std::array<double, 3> minimumsPlusEpsilon = {1.1 + delta, 4.4 + delta, -7.2 + delta};
	std::array<double, 3> maximumsPlusEpsilon = {4.1 + delta, 3.2 + delta, -1.0 + delta};
	std::array<double, 3> maximumsPlus2Epsilon = {4.1 + (2 * epsilon), 3.2 + delta, -1.0 + delta};

	std::array<Interval<double>, 3> testIntervalArray;
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
	EXPECT_NO_THROW(testInterval3DMoveConstructor(b));
	IntervalDouble3 f = testInterval3DMoveConstructor(b);
	EXPECT_TRUE(f == b);

	// Assignment and equal/not equal tests
	IntervalDouble3 c;
	EXPECT_TRUE(c != b);
	EXPECT_FALSE(c == b);
	c = a;
	EXPECT_FALSE(c != b);
	EXPECT_TRUE(c == b);
	IntervalDouble3 e;
	e = (b * c);
	EXPECT_TRUE(e == (b * c));

	IntervalDouble3 bPlusEpsilon(minimumsPlusEpsilon, maximumsPlusEpsilon);
	IntervalDouble3 bPlus2Epsilon(minimumsPlusEpsilon, maximumsPlus2Epsilon);
	EXPECT_TRUE(b.isApprox(bPlusEpsilon, epsilon));
	EXPECT_TRUE(bPlusEpsilon.isApprox(b, epsilon));
	EXPECT_FALSE(b.isApprox(bPlus2Epsilon, epsilon));
	EXPECT_FALSE(bPlus2Epsilon.isApprox(b, epsilon));
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
	std::array<double, 3> minimumsShape = {1.1, 3.2, -7.2};
	std::array<double, 3> maximumsShape = {4.1, 4.4, -1.0};

	std::array<double, 3> noOverlapMin = {1.1, 4.5, -8.0};
	std::array<double, 3> noOverlapMax = {4.5, 4.8, -3.0};

	std::array<double, 3> overlapMin = {1.7, 2.0, -8.0};
	std::array<double, 3> overlapMax = {4.3, 4.7, -3.0};

	IntervalDouble3 shape(minimumsShape, maximumsShape);

	EXPECT_FALSE(shape.overlapsWith(IntervalDouble3(noOverlapMin, noOverlapMax)));
	EXPECT_TRUE(shape.overlapsWith(IntervalDouble3(overlapMin, overlapMax)));
};

TEST_F(IntervalArithmeticTests, Interval3DExtendRangeTests)
{
	std::array<double, 3> minimumsShape = {1.1, 3.2, -7.2};
	std::array<double, 3> maximumsShape = {4.1, 4.4, -1.0};

	std::array<double, 3> finalMin = { -5.9, -3.8, -14.2};
	std::array<double, 3> finalMax = {11.1, 11.4, 6.0};

	IntervalDouble3 shape(minimumsShape, maximumsShape);
	shape.addThickness(7.0);
	EXPECT_EQ(IntervalDouble3(finalMin, finalMax), shape);
};

TEST_F(IntervalArithmeticTests, Interval3DOperatorTests)
{
	std::array<double, 3> minimumsShape = {1.1, 3.2, -7.2};
	std::array<double, 3> maximumsShape = {4.1, 4.4, -1.0};

	std::array<double, 3> deltaMin = { -1.0, -2.0, -3.0};
	std::array<double, 3> deltaMax = {3.0, 4.0, 5.0};

	std::array<double, 3> nonZeroDeltaMin = { -3.0, 2.0, 3.0};
	std::array<double, 3> nonZeroDeltaMax = { -1.0, 4.0, 5.0};

	IntervalDouble3 initial(minimumsShape, maximumsShape);
	IntervalDouble3 delta(deltaMin, deltaMax);
	IntervalDouble3 nonZeroDelta(nonZeroDeltaMin, nonZeroDeltaMax);
	IntervalDouble3 dummy;

	// +
	EXPECT_TRUE((Interval<double>(1.1, 4.1) + Interval<double>(-1.0, 3.0)).isApprox(
					(initial + delta).getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) + Interval<double>(-2.0, 4.0)).isApprox(
					(initial + delta).getAxis(1), epsilon));
	EXPECT_TRUE((Interval<double>(-7.2, -1.0) + Interval<double>(-3.0, 5.0)).isApprox(
					(initial + delta).getAxis(2), epsilon));

	// +=
	dummy = initial;
	dummy += delta;
	EXPECT_TRUE((Interval<double>(1.1, 4.1) + Interval<double>(-1.0, 3.0)).isApprox(dummy.getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) + Interval<double>(-2.0, 4.0)).isApprox(dummy.getAxis(1), epsilon));
	EXPECT_TRUE((Interval<double>(-7.2, -1.0) + Interval<double>(-3.0, 5.0)).isApprox(dummy.getAxis(2), epsilon));

	// -
	EXPECT_TRUE((Interval<double>(1.1, 4.1) - Interval<double>(-1.0, 3.0)).isApprox(
					(initial - delta).getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) - Interval<double>(-2.0, 4.0)).isApprox(
					(initial - delta).getAxis(1), epsilon));
	EXPECT_TRUE((Interval<double>(-7.2, -1.0) - Interval<double>(-3.0, 5.0)).isApprox(
					(initial - delta).getAxis(2), epsilon));

	// -=
	dummy = initial;
	dummy -= delta;
	EXPECT_TRUE((Interval<double>(1.1, 4.1) - Interval<double>(-1.0, 3.0)).isApprox(dummy.getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) - Interval<double>(-2.0, 4.0)).isApprox(dummy.getAxis(1), epsilon));
	EXPECT_TRUE((Interval<double>(-7.2, -1.0) - Interval<double>(-3.0, 5.0)).isApprox(dummy.getAxis(2), epsilon));

	// *
	EXPECT_TRUE((Interval<double>(1.1, 4.1) * Interval<double>(-1.0, 3.0)).isApprox(
					(initial * delta).getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) * Interval<double>(-2.0, 4.0)).isApprox(
					(initial * delta).getAxis(1), epsilon));
	EXPECT_TRUE((Interval<double>(-7.2, -1.0) * Interval<double>(-3.0, 5.0)).isApprox(
					(initial * delta).getAxis(2), epsilon));

	// *=
	dummy = initial;
	dummy *= delta;
	EXPECT_TRUE((Interval<double>(1.1, 4.1) * Interval<double>(-1.0, 3.0)).isApprox(dummy.getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) * Interval<double>(-2.0, 4.0)).isApprox(dummy.getAxis(1), epsilon));
	EXPECT_TRUE((Interval<double>(-7.2, -1.0) * Interval<double>(-3.0, 5.0)).isApprox(dummy.getAxis(2), epsilon));

	// /
	EXPECT_TRUE((Interval<double>(1.1, 4.1) / Interval<double>(-3.0, -1.0)).isApprox(
					(initial / nonZeroDelta).getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) / Interval<double>(2.0, 4.0)).isApprox(
					(initial / nonZeroDelta).getAxis(1), epsilon));
	EXPECT_TRUE((Interval<double>(-7.2, -1.0) / Interval<double>(3.0, 5.0)).isApprox(
					(initial / nonZeroDelta).getAxis(2), epsilon));

	// /=
	dummy = initial;
	dummy /= nonZeroDelta;
	EXPECT_TRUE((Interval<double>(1.1, 4.1) / Interval<double>(-3.0, -1.0)).isApprox(dummy.getAxis(0), epsilon));
	EXPECT_TRUE((Interval<double>(3.2, 4.4) / Interval<double>(2.0, 4.0)).isApprox(dummy.getAxis(1), epsilon));
	EXPECT_TRUE((Interval<double>(-7.2, -1.0) / Interval<double>(3.0, 5.0)).isApprox(dummy.getAxis(2), epsilon));

	// Inverse
	EXPECT_TRUE(Interval<double>(1.1, 4.1).inverse().isApprox(initial.inverse().getAxis(0), epsilon));
	EXPECT_TRUE(Interval<double>(3.2, 4.4).inverse().isApprox(initial.inverse().getAxis(1), epsilon));
	EXPECT_TRUE(Interval<double>(-7.2, -1.0).inverse().isApprox(initial.inverse().getAxis(2), epsilon));

	// Dot product
	Interval<double> dotValue = initial.dotProduct(delta);
	EXPECT_TRUE(dotValue.isApprox((initial.getAxis(0) * delta.getAxis(0)) +
								  (initial.getAxis(1) * delta.getAxis(1)) +
								  (initial.getAxis(2) * delta.getAxis(2)), epsilon));

	// Cross product
	IntervalDouble3 crossValue = initial.crossProduct(delta);
	EXPECT_TRUE(crossValue.getAxis(0).isApprox((initial.getAxis(1) * delta.getAxis(2) -
				initial.getAxis(2) * delta.getAxis(1)), epsilon));
	EXPECT_TRUE(crossValue.getAxis(1).isApprox((initial.getAxis(2) * delta.getAxis(0) -
				initial.getAxis(0) * delta.getAxis(2)), epsilon));
	EXPECT_TRUE(crossValue.getAxis(2).isApprox((initial.getAxis(0) * delta.getAxis(1) -
				initial.getAxis(1) * delta.getAxis(0)), epsilon));

	// Magnitude and magnitude squared
	Interval<double> magnitudeSquaredValue = initial.magnitudeSquared();
	EXPECT_TRUE(magnitudeSquaredValue.isApprox((initial.getAxis(0).square() +
				initial.getAxis(1).square() + initial.getAxis(2).square()), epsilon));

	Interval<double> magnitudeValue = initial.magnitude();
	EXPECT_NEAR(std::sqrt(magnitudeSquaredValue.getMin()), magnitudeValue.getMin(), epsilon);
	EXPECT_NEAR(std::sqrt(magnitudeSquaredValue.getMax()), magnitudeValue.getMax(), epsilon);
};

TEST_F(IntervalArithmeticTests, IntervalUtilityTests)
{
	Interval<double> initial(10.0, 20.0);
	Interval<double> deltaInterval(3.0, 4.0);
	Interval<double> resultLoad(-2.0, -1.0);
	Interval<double> result;

	// Constant + interval
	EXPECT_TRUE((initial + 11.5).isApprox(11.5 + initial, epsilon));

	// Constant * interval
	EXPECT_TRUE((initial * 11.5).isApprox(11.5 * initial, epsilon));

	// Output
	std::ostringstream intervalOutput;
	intervalOutput << initial;
	EXPECT_EQ("[10,20]", intervalOutput.str());

	// +
	IntervalArithmetic_add(initial, deltaInterval, &result);
	EXPECT_TRUE((initial + deltaInterval).isApprox(result, epsilon));

	// +=( + )
	result = resultLoad;
	IntervalArithmetic_addadd(initial, deltaInterval, &result);
	EXPECT_TRUE((resultLoad + initial + deltaInterval).isApprox(result, epsilon));

	// -
	IntervalArithmetic_sub(initial, deltaInterval, &result);
	EXPECT_TRUE((initial - deltaInterval).isApprox(result, epsilon));

	// +=( - )
	result = resultLoad;
	IntervalArithmetic_addsub(initial, deltaInterval, &result);
	EXPECT_TRUE((resultLoad + (initial - deltaInterval)).isApprox(result, epsilon));

	// *
	IntervalArithmetic_mul(initial, deltaInterval, &result);
	EXPECT_TRUE((initial * deltaInterval).isApprox(result, epsilon));

	// += ( * )
	result = resultLoad;
	IntervalArithmetic_addmul(initial, deltaInterval, &result);
	EXPECT_TRUE((resultLoad + (initial * deltaInterval)).isApprox(result, epsilon));

	// -= ( * )
	result = resultLoad;
	IntervalArithmetic_submul(initial, deltaInterval, &result);
	EXPECT_TRUE((resultLoad - (initial * deltaInterval)).isApprox(result, epsilon));
};

TEST_F(IntervalArithmeticTests, IntervalNDUtilityTests)
{
	std::array<double, 2> minimumsShape = {1.1, 3.2};
	std::array<double, 2> maximumsShape = {4.1, 4.4};

	IntervalDouble2 shape(minimumsShape, maximumsShape);

	// Output
	std::ostringstream intervalOutput;
	intervalOutput << shape;
	EXPECT_EQ("([1.1,4.1];[3.2,4.4])", intervalOutput.str());
};

TEST_F(IntervalArithmeticTests, Interval3DUtilityTests)
{
	std::array<double, 3> minimumsShape = {1.1, 3.2, -7.2};
	std::array<double, 3> maximumsShape = {4.1, 4.4, -1.0};

	std::array<double, 3> deltaMin = { -1.0, -2.0, -3.0};
	std::array<double, 3> deltaMax = {3.0, 4.0, 5.0};

	IntervalDouble3 initial(minimumsShape, maximumsShape);
	IntervalDouble3 deltaInterval(deltaMin, deltaMax);
	IntervalDouble3 result;
	Interval<double> intervalResult;

	// +
	IntervalArithmetic_add(initial, deltaInterval, &result);
	EXPECT_TRUE((initial + deltaInterval).isApprox(result, epsilon));

	// -
	IntervalArithmetic_sub(initial, deltaInterval, &result);
	EXPECT_TRUE((initial - deltaInterval).isApprox(result, epsilon));

	// dot product
	IntervalArithmetic_dotProduct(initial, deltaInterval, &intervalResult);
	EXPECT_TRUE(initial.dotProduct(deltaInterval).isApprox(intervalResult, epsilon));

	// Cross product
	IntervalArithmetic_crossProduct(initial, deltaInterval, &result);
	EXPECT_TRUE(initial.crossProduct(deltaInterval).isApprox(result, epsilon));
};

}; // namespace Math

}; // namespace SurgSim

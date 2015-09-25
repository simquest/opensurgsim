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
/// Tests for the SegmentSegmentCcdCheck functions.

#include <array>

#include <gtest/gtest.h>

#include "SurgSim/Collision/SegmentSegmentCcdIntervalCheck.h"
#include "SurgSim/Math/LinearMotionArithmetic.h"

namespace SurgSim
{

namespace Collision
{

namespace
{
double epsilon = 1.0e-10;
}

template <typename T>
class SegmentSegmentCcdIntervalCheckTests : public ::testing::Test
{
};

TEST(SegmentSegmentCcdIntervalCheckTests, Initialization)
{
	// Set up some arbitrary locations to check the class initialization.
	std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, 1.0, 0.0), Math::Vector3d(-0.5, 1.5, 1.5)};
	std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, 1.0, 1.5), Math::Vector3d(-0.5, 1.5, 3.0)};
	std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(2.0, 1.0, 4.0), Math::Vector3d(1.0, 3.5, 2.5)};
	std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(0.0, 1.0, 2.0), Math::Vector3d(-2.0, 1.5, 3.5)};

	ASSERT_NO_THROW(SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd,
					1.0e-06, 2.0e-06, 3.0e-06, 4.0e-06));
	SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 1.0e-06, 2.0e-06, 3.0e-06, 4.0e-06);

	Math::Vector3d p11 = status.p1T0();
	Math::Vector3d p12 = status.p1T1();
	Math::Vector3d p21 = status.p2T0();
	Math::Vector3d p22 = status.p2T1();
	Math::Vector3d q11 = status.q1T0();
	Math::Vector3d q12 = status.q1T1();
	Math::Vector3d q21 = status.q2T0();
	Math::Vector3d q22 = status.q2T1();

	EXPECT_TRUE(p11.isApprox(Math::Vector3d(-1.0, 1.0, 0.0), epsilon));
	EXPECT_TRUE(p12.isApprox(Math::Vector3d(-1.0, 1.0, 1.5), epsilon));
	EXPECT_TRUE(p21.isApprox(Math::Vector3d(-0.5, 1.5, 1.5), epsilon));
	EXPECT_TRUE(p22.isApprox(Math::Vector3d(-0.5, 1.5, 3.0), epsilon));
	EXPECT_TRUE(q11.isApprox(Math::Vector3d(2.0, 1.0, 4.0), epsilon));
	EXPECT_TRUE(q12.isApprox(Math::Vector3d(0.0, 1.0, 2.0), epsilon));
	EXPECT_TRUE(q21.isApprox(Math::Vector3d(1.0, 3.5, 2.5), epsilon));
	EXPECT_TRUE(q22.isApprox(Math::Vector3d(-2.0, 1.5, 3.5), epsilon));

	EXPECT_TRUE((Math::LinearMotionND<double, 3>(p11, p12)).isApprox(status.motionP1(), epsilon));
	EXPECT_TRUE((Math::LinearMotionND<double, 3>(p21, p22)).isApprox(status.motionP2(), epsilon));
	EXPECT_TRUE((Math::LinearMotionND<double, 3>(q11, q12)).isApprox(status.motionQ1(), epsilon));
	EXPECT_TRUE((Math::LinearMotionND<double, 3>(q21, q22)).isApprox(status.motionQ2(), epsilon));

	EXPECT_DOUBLE_EQ(1.0e-06, status.thicknessP());
	EXPECT_DOUBLE_EQ(2.0e-06, status.thicknessQ());
	EXPECT_DOUBLE_EQ(3.0e-06, status.timePrecisionEpsilon());
	EXPECT_DOUBLE_EQ(4.0e-06, status.distanceEpsilon());
	EXPECT_DOUBLE_EQ(0.0, status.tripleProductEpsilon());
	EXPECT_DOUBLE_EQ(0.0, status.muNuEpsilon());


	auto p1q1 = status.motionQ1() - status.motionP1();
	auto p1p2 = status.motionP2() - status.motionP1();
	auto q1q2 = status.motionQ2() - status.motionQ1();
	EXPECT_TRUE(analyticTripleProduct(p1q1, p1p2, q1q2).isApprox(status.P1Q1_P1P2_Q1Q2().getPolynomial(), epsilon));
	EXPECT_TRUE(analyticDotProduct(p1p2, p1q1).isApprox(status.P1P2_P1Q1().getPolynomial(), epsilon));
	EXPECT_TRUE(analyticDotProduct(q1q2, p1q1).isApprox(status.Q1Q2_P1Q1().getPolynomial(), epsilon));
	EXPECT_TRUE(analyticDotProduct(p1p2, q1q2).isApprox(status.P1P2_Q1Q2().getPolynomial(), epsilon));
	EXPECT_TRUE(analyticMagnitudeSquared(p1p2).isApprox(status.P1P2_sq().getPolynomial(), epsilon));
	EXPECT_TRUE(analyticMagnitudeSquared(q1q2).isApprox(status.Q1Q2_sq().getPolynomial(), epsilon));

	Math::Interval<double> range(0.5, 0.75);
	Math::Polynomial<double, 2> axisX;
	Math::Polynomial<double, 2> axisY;
	Math::Polynomial<double, 2> axisZ;

	Math::analyticCrossProduct(p1p2, q1q2, &axisX, &axisY, &axisZ);
	Math::PolynomialValues<double, 2> valueX(axisX);
	Math::PolynomialValues<double, 2> valueY(axisY);
	Math::PolynomialValues<double, 2> valueZ(axisZ);
	Math::Interval<double> cross = valueX.valuesOverInterval(range).square() +
								   valueY.valuesOverInterval(range).square() +
								   valueZ.valuesOverInterval(range).square();
	EXPECT_TRUE(cross.isApprox(status.crossValueOnInterval(range), epsilon));
};

TEST(SegmentSegmentCcdIntervalCheckTests, SetGetTests)
{
	// Set up some arbitrary locations to check the class initialization.
	std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, 1.0, 0.0), Math::Vector3d(-0.5, 1.5, 1.5)};
	std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, 1.0, 1.5), Math::Vector3d(-0.5, 1.5, 3.0)};
	std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(2.0, 1.0, 4.0), Math::Vector3d(1.0, 3.5, 2.5)};
	std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(0.0, 1.0, 2.0), Math::Vector3d(-2.0, 1.5, 3.5)};

	SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 1.0e-06, 2.0e-06, 3.0e-06, 4.0e-06);

	status.setTimePrecisionEpsilon(5.0e-06);
	status.setDistanceEpsilon(6.0e-06);
	status.setTripleProductEpsilon(7.0e-06);
	status.setMuNuEpsilon(8.0e-06);

	EXPECT_DOUBLE_EQ(5.0e-06, status.timePrecisionEpsilon());
	EXPECT_DOUBLE_EQ(6.0e-06, status.distanceEpsilon());
	EXPECT_DOUBLE_EQ(7.0e-06, status.tripleProductEpsilon());
	EXPECT_DOUBLE_EQ(8.0e-06, status.muNuEpsilon());
};

TEST(SegmentSegmentCcdIntervalCheckTests, CollisionChecksWithThickness)
{
	{
		// Two crossed segments intersecting at X = Y = 0 when the distance between is <= 0.75. This
		// should occur when t >= 0.75
		SCOPED_TRACE("Testing simple crossed segments moving towards each other.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(1.0, -1.0, 1.0), Math::Vector3d(-1.0, 1.0, 1.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(1.0, -1.0, 0.0), Math::Vector3d(-1.0, 1.0, 0.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);

		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 0.7500000001)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 0.7499999999)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 0.5)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.5, 0.7499999999)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.5, 0.7500000001)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.7499999999, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.7500000001, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.9, 0.95)));
	}
	{
		// Two crossed segments intersecting at X = Y = 0 when the distance between is <= 0.75. This
		// should occur when t <= 0.25
		SCOPED_TRACE("Testing simple crossed segments moving away from each other.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(1.0, -1.0, 0.0), Math::Vector3d(-1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(1.0, -1.0, 1.0), Math::Vector3d(-1.0, 1.0, 1.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);

		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.2499999999, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.2499999999, 0.2500000001)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.2500000001, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.5, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.5, 0.7499999999)));
	}
	{
		SCOPED_TRACE("Slanted T formation around Q1. Segments never get close enough.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> qStart =
		{
			Math::Vector3d(0.0, 0.7500000001, 1.0),
			Math::Vector3d(0.0, 1.7500000001, 1.0)
		};
		std::array<Math::Vector3d, 2> qEnd =
		{
			Math::Vector3d(0.0, 0.7500000001, 0.0),
			Math::Vector3d(0.0, 1.7500000001, 0.0)
		};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionEndpoints,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around Q1. Segments get close enough right around time = 1.0.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> qStart =
		{
			Math::Vector3d(0.0, 0.7499999999, 1.0),
			Math::Vector3d(0.0, 1.7499999999, 1.0)
		};
		std::array<Math::Vector3d, 2> qEnd =
		{
			Math::Vector3d(0.0, 0.7499999999, 0.0),
			Math::Vector3d(0.0, 1.7499999999, 0.0)
		};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around Q2. Segments never get close enough.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> qStart =
		{
			Math::Vector3d(0.0, -1.7500000001, 1.0),
			Math::Vector3d(0.0, -0.7500000001, 1.0)
		};
		std::array<Math::Vector3d, 2> qEnd =
		{
			Math::Vector3d(0.0, -1.7500000001, 0.0),
			Math::Vector3d(0.0, -0.7500000001, 0.0)
		};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionEndpoints,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around Q2. Segments get close enough right around time = 1.0.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> qStart =
		{
			Math::Vector3d(0.0, -1.7499999999, 1.0),
			Math::Vector3d(0.0, -0.7499999999, 1.0)
		};
		std::array<Math::Vector3d, 2> qEnd =
		{
			Math::Vector3d(0.0, -1.7499999999, 0.0),
			Math::Vector3d(0.0, -0.7499999999, 0.0)
		};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around P1. Segments never get close enough.");
		std::array<Math::Vector3d, 2> pStart =
		{
			Math::Vector3d(0.0, 0.7500000001, 1.0),
			Math::Vector3d(0.0, 1.7500000001, 1.0)
		};
		std::array<Math::Vector3d, 2> pEnd =
		{
			Math::Vector3d(0.0, 0.7500000001, 0.0),
			Math::Vector3d(0.0, 1.7500000001, 0.0)
		};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionEndpoints,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around P1. Segments get close enough right around time = 1.0.");
		std::array<Math::Vector3d, 2> pStart =
		{
			Math::Vector3d(0.0, 0.7499999999, 1.0),
			Math::Vector3d(0.0, 1.7499999999, 1.0)
		};
		std::array<Math::Vector3d, 2> pEnd =
		{
			Math::Vector3d(0.0, 0.7499999999, 0.0),
			Math::Vector3d(0.0, 1.7499999999, 0.0)
		};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around P2. Segments never get close enough.");
		std::array<Math::Vector3d, 2> pStart =
		{
			Math::Vector3d(0.0, -1.7500000001, 1.0),
			Math::Vector3d(0.0, -0.7500000001, 1.0)
		};
		std::array<Math::Vector3d, 2> pEnd =
		{
			Math::Vector3d(0.0, -1.7500000001, 0.0),
			Math::Vector3d(0.0, -0.7500000001, 0.0)
		};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionEndpoints,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around P2. Segments get close enough right around time = 1.0.");
		std::array<Math::Vector3d, 2> pStart =
		{
			Math::Vector3d(0.0, -1.7499999999, 1.0),
			Math::Vector3d(0.0, -0.7499999999, 1.0)
		};
		std::array<Math::Vector3d, 2> pEnd =
		{
			Math::Vector3d(0.0, -1.7499999999, 0.0),
			Math::Vector3d(0.0, -0.7499999999, 0.0)
		};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestWithThickness(Math::Interval<double> (0.0, 1.0)));
	}
};

TEST(SegmentSegmentCcdIntervalCheckTests, CollisionChecksWithoutThickness)
{
	{
		// Two crossed segments intersecting at X = Y = 0 when the distance between is = 0.0. This
		// should occur when t >= 0.5
		SCOPED_TRACE("Testing simple crossed segments moving towards each other.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 2.0), Math::Vector3d(1.0, 1.0, 2.0)};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(1.0, -1.0, 1.0), Math::Vector3d(-1.0, 1.0, 1.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(1.0, -1.0, -1.0), Math::Vector3d(-1.0, 1.0, -1.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);

		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 0.5000000001)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 0.4999999999)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 0.25)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.25, 0.4999999999)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.4999999999, 0.5000000001)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.4999999999, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.5000000001, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.9, 0.95)));
	}
	{
		// Two crossed segments intersecting at X = Y = 0 when the distance between is = 0.0. This
		// should occur at t = 0.0
		SCOPED_TRACE("Testing simple crossed segments moving away from each other.");
		std::array<Math::Vector3d, 2> pStart =
		{
			Math::Vector3d(-1.0, -1.0, 0.0000000001), Math::Vector3d(1.0, 1.0, 0.0000000001)
		};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> qStart =
		{
			Math::Vector3d(1.0, -1.0, -0.0000000001), Math::Vector3d(-1.0, 1.0, -0.0000000001)
		};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(1.0, -1.0, 1.0), Math::Vector3d(-1.0, 1.0, 1.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);

		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.25, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 0.1)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.000001, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.5, 1.0)));
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionVolume,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.5, 0.7499999999)));
	}
	{
		SCOPED_TRACE("Slanted T formation around Q1. Segments never get close enough.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> qStart =
		{
			Math::Vector3d(0.0, 0.0000000001, 1.0),
			Math::Vector3d(0.0, 1.0000000001, 1.0)
		};
		std::array<Math::Vector3d, 2> qEnd =
		{
			Math::Vector3d(0.0, 0.0000000001, 0.0),
			Math::Vector3d(0.0, 1.0000000001, 0.0)
		};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionEndpoints,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around Q1. Segments get close enough right around time = 1.0.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> qStart =
		{
			Math::Vector3d(0.0, -0.0000000001, 1.0),
			Math::Vector3d(0.0, 1.0000000001, 1.0)
		};
		std::array<Math::Vector3d, 2> qEnd =
		{
			Math::Vector3d(0.0, -0.0000000001, 0.0),
			Math::Vector3d(0.0, 1.0000000001, 0.0)
		};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around Q2. Segments never get close enough.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> qStart =
		{
			Math::Vector3d(0.0, -1.0000000001, 1.0),
			Math::Vector3d(0.0, -0.0000000001, 1.0)
		};
		std::array<Math::Vector3d, 2> qEnd =
		{
			Math::Vector3d(0.0, -1.0000000001, 0.0),
			Math::Vector3d(0.0, -0.0000000001, 0.0)
		};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionEndpoints,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around Q2. Segments get close enough right around time = 1.0.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> qStart =
		{
			Math::Vector3d(0.0, -1.0000000001, 1.0),
			Math::Vector3d(0.0, 0.0000000001, 1.0)
		};
		std::array<Math::Vector3d, 2> qEnd =
		{
			Math::Vector3d(0.0, -1.0000000001, 0.0),
			Math::Vector3d(0.0, 0.0000000001, 0.0)
		};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around P1. Segments never get close enough.");
		std::array<Math::Vector3d, 2> pStart =
		{
			Math::Vector3d(0.0, 0.0000000001, 1.0),
			Math::Vector3d(0.0, 1.0000000001, 1.0)
		};
		std::array<Math::Vector3d, 2> pEnd =
		{
			Math::Vector3d(0.0, 0.0000000001, 0.0),
			Math::Vector3d(0.0, 1.0000000001, 0.0)
		};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionEndpoints,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around P1. Segments get close enough right around time = 1.0.");
		std::array<Math::Vector3d, 2> pStart =
		{
			Math::Vector3d(0.0, -0.0000000001, 1.0),
			Math::Vector3d(0.0, 1.0000000001, 1.0)
		};
		std::array<Math::Vector3d, 2> pEnd =
		{
			Math::Vector3d(0.0, -0.0000000001, 0.0),
			Math::Vector3d(0.0, 1.0000000001, 0.0)
		};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around P2. Segments never get close enough.");
		std::array<Math::Vector3d, 2> pStart =
		{
			Math::Vector3d(0.0, -1.0000000001, 1.0),
			Math::Vector3d(0.0, -0.0000000001, 1.0)
		};
		std::array<Math::Vector3d, 2> pEnd =
		{
			Math::Vector3d(0.0, -1.0000000001, 0.0),
			Math::Vector3d(0.0, -0.0000000001, 0.0)
		};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckNoCollisionEndpoints,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 1.0)));
	}
	{
		SCOPED_TRACE("Slanted T formation around P2. Segments get close enough right around time = 1.0.");
		std::array<Math::Vector3d, 2> pStart =
		{
			Math::Vector3d(0.0, -1.0000000001, 1.0),
			Math::Vector3d(0.0, 0.0000000001, 1.0)
		};
		std::array<Math::Vector3d, 2> pEnd =
		{
			Math::Vector3d(0.0, -1.0000000001, 0.0),
			Math::Vector3d(0.0, 0.0000000001, 0.0)
		};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};

		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_EQ(SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision,
				  status.possibleCollisionTestNoThickness(Math::Interval<double> (0.0, 1.0)));
	}
};

}; // namespace Collision
}; // namespace SurgSim

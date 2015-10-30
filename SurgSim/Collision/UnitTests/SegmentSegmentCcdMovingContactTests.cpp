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
/// Tests for the SegmentSegmentCcdMovingContact functions.

#include <array>

#include <gtest/gtest.h>

#include "SurgSim/Collision/SegmentSegmentCcdMovingContact.h"

namespace SurgSim
{

namespace Collision
{

namespace
{
double epsilon = 1.0e-05;
}

template <typename T>
class SegmentSegmentCcdMovingContactTests : public ::testing::Test
{
};

class MockSegmentSegmentCcdMovingContact : public SurgSim::Collision::SegmentSegmentCcdMovingContact
{
public:
	void normalizeSegmentsConsistently(Math::Vector3d* t0, Math::Vector3d* t1, double epsilon) const
	{
		SegmentSegmentCcdMovingContact::normalizeSegmentsConsistently(t0, t1, epsilon);
	}

	bool collideSegmentSegmentBaseCase(
		const std::array<Math::Vector3d, 2>& pT0,
		const std::array<Math::Vector3d, 2>& pT1,
		const std::array<Math::Vector3d, 2>& qT0,
		const std::array<Math::Vector3d, 2>& qT1,
		double thicknessP,
		double thicknessQ,
		double timePrecisionEpsilon,
		double* t, double* r, double* s)
	{
		return SegmentSegmentCcdMovingContact::collideSegmentSegmentBaseCase(pT0, pT1, qT0, qT1,
				thicknessP, thicknessQ, timePrecisionEpsilon,
				t, r, s);
	}

	bool collideSegmentSegmentGeneralCase(
		const SegmentSegmentCcdIntervalCheck& state,
		double a, double b, // Interval boundaries
		double* t, double* r, double* s,
		int depth = 0)
	{
		return SegmentSegmentCcdMovingContact::collideSegmentSegmentGeneralCase(state, a, b, t, r, s, depth);
	}

	bool collideSegmentSegmentCoplanarCase(
		const std::array<Math::Vector3d, 2>& pT0, /* Segment 1 at t=0 */
		const std::array<Math::Vector3d, 2>& pT1, /* Segment 1 at t=1 */
		const std::array<Math::Vector3d, 2>& qT0, /* Segment 2 at t=0 */
		const std::array<Math::Vector3d, 2>& qT1, /* Segment 2 at t=1 */
		double a, double b, /* Interval boundaries */
		double timePrecisionEpsilon,
		double thicknessP, double thicknessQ,
		double* t, double* r, double* s,
		int depth = 0)
	{
		m_staticTest.collideStaticSegmentSegment(pT0, qT0, thicknessP, thicknessQ, r, s);
		return SegmentSegmentCcdMovingContact::collideSegmentSegmentCoplanarCase(
				   pT0, pT1, qT0, qT1, a, b, timePrecisionEpsilon, thicknessP, thicknessQ, t, r, s, depth);
	}

	bool collideSegmentSegmentParallelCase(
		const std::array<Math::Vector3d, 2>& pT0,
		const std::array<Math::Vector3d, 2>& pT1,
		const std::array<Math::Vector3d, 2>& qT0,
		const std::array<Math::Vector3d, 2>& qT1,
		double a, double b,
		double thicknessP, double thicknessQ,
		double timePrecisionEpsilon,
		double* t, double* r, double* s, int depth = 0)
	{
		m_staticTest.collideStaticSegmentSegment(pT0, qT0, thicknessP, thicknessQ, r, s);
		return SegmentSegmentCcdMovingContact::collideSegmentSegmentParallelCase(
				   pT0, pT1, qT0, qT1, a, b, thicknessP, thicknessQ, timePrecisionEpsilon, t, r, s, depth);

	}

	Collision::SegmentSegmentCcdStaticContact m_staticTest;
};

TEST(SegmentSegmentCcdMovingContactTests, Initialization)
{
	ASSERT_NO_THROW(SegmentSegmentCcdMovingContact movingTest);
	SegmentSegmentCcdMovingContact movingTest;
};

TEST(SegmentSegmentCcdMovingContactTests, TestSafeNormalization)
{
	MockSegmentSegmentCcdMovingContact movingTest;

	{
		SCOPED_TRACE("Normalizing 2 good vectors.");
		Math::Vector3d vector1(0.0, 0.1, 0.2);
		Math::Vector3d vector2(0.2, 0.1, 0.0);
		movingTest.normalizeSegmentsConsistently(&vector1, &vector2, epsilon);
		EXPECT_TRUE(vector1.isApprox(Math::Vector3d(0.0, 1.0 / std::sqrt(5.0), 2 / std::sqrt(5.0)), epsilon));
		EXPECT_TRUE(vector2.isApprox(Math::Vector3d(2 / std::sqrt(5.0), 1.0 / std::sqrt(5.0), 0.0), epsilon));
	}
	{
		SCOPED_TRACE("Vector 1 is good. Vector 2 is bad.");
		Math::Vector3d vector1(0.0, 0.1, 0.2);
		Math::Vector3d vector2(2.0e-11, 1.0e-11, 0.0);
		movingTest.normalizeSegmentsConsistently(&vector1, &vector2, epsilon);
		EXPECT_TRUE(vector1.isApprox(Math::Vector3d(0.0, 1.0 / std::sqrt(5.0), 2 / std::sqrt(5.0)), epsilon));
		EXPECT_TRUE(vector2.isApprox(Math::Vector3d(0.0, 1.0 / std::sqrt(5.0), 2 / std::sqrt(5.0)), epsilon));
	}
	{
		SCOPED_TRACE("Vector 1 is bad. Vector 2 is good.");
		Math::Vector3d vector1(0.0, 1.0e-11, 2.0e-11);
		Math::Vector3d vector2(0.2, 0.1, 0.0);
		movingTest.normalizeSegmentsConsistently(&vector1, &vector2, epsilon);
		EXPECT_TRUE(vector1.isApprox(Math::Vector3d(2 / std::sqrt(5.0), 1.0 / std::sqrt(5.0), 0.0), epsilon));
		EXPECT_TRUE(vector2.isApprox(Math::Vector3d(2 / std::sqrt(5.0), 1.0 / std::sqrt(5.0), 0.0), epsilon));
	}
	{
		SCOPED_TRACE("Normalizing 2 bad vectors.");
		Math::Vector3d vector1(0.0, 1.0e-11, 2.0e-11);
		Math::Vector3d vector2(2.0e-11, 1.0e-11, 0.0);
		movingTest.normalizeSegmentsConsistently(&vector1, &vector2, epsilon);
		EXPECT_TRUE(vector1.isApprox(Math::Vector3d(0.0, 1.0e-11, 2.0e-11), epsilon));
		EXPECT_TRUE(vector2.isApprox(Math::Vector3d(2.0e-11, 1.0e-11, 0.0), epsilon));
	}
};

TEST(SegmentSegmentCcdMovingContactTests, TestSegmentSegmentGeneralCase)
{
	MockSegmentSegmentCcdMovingContact movingTest;

	std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
	std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
	std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(1.0, -1.0, 1.0), Math::Vector3d(-1.0, 1.0, 1.0)};
	std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(1.0, -1.0, 0.0), Math::Vector3d(-1.0, 1.0, 0.0)};

	double t;
	double r;
	double s;

	{
		SCOPED_TRACE("Collision not possible within time interval.");
		SegmentSegmentCcdIntervalCheck status(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_FALSE(movingTest.collideSegmentSegmentGeneralCase(status, 0.0, 0.5, &t, &r, &s, 0));
	}

	{
		// Test where collision occurs at the end of the (small) time interval
		SCOPED_TRACE("Recursion bottomed out, test collision and no collision at time precision.");

		SegmentSegmentCcdIntervalCheck statusTrue(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_TRUE(movingTest.collideSegmentSegmentGeneralCase(statusTrue, 0.75 - 1.4e-07, 0.75 + 1.5e-07,
					&t, &r, &s, 0));
		EXPECT_DOUBLE_EQ(0.75 + 1.5e-07, t);
		EXPECT_DOUBLE_EQ(0.50, r);
		EXPECT_DOUBLE_EQ(0.50, s);

		// Increase the interval and time precision so that no collision occurs
		SegmentSegmentCcdIntervalCheck statusFalse(pStart, pEnd, qStart, qEnd, 1.0e-06, 1.0e-06, 0.5, 4.0e-06);
		EXPECT_FALSE(movingTest.collideSegmentSegmentGeneralCase(statusFalse, 0.76 - 2.5, 0.75 + 2.4999,
					 &t, &r, &s, 0));
	}

	{
		// Test complete recursion
		SCOPED_TRACE("Full recursion test, test collision and no collision at time precision.");

		SegmentSegmentCcdIntervalCheck statusTrue(pStart, pEnd, qStart, qEnd, 0.5, 0.25, 3.0e-06, 4.0e-06);
		EXPECT_TRUE(movingTest.collideSegmentSegmentGeneralCase(statusTrue, 0.0, 1.0, &t, &r, &s, 0));
		EXPECT_GT(epsilon, std::abs(0.75 - t));
		EXPECT_DOUBLE_EQ(0.50, r);
		EXPECT_DOUBLE_EQ(0.50, s);

		// Increase the interval and time precision so that no collision occurs
		SegmentSegmentCcdIntervalCheck statusFalse(pStart, pEnd, qStart, qEnd, 1.0e-06, 1.0e-06, 0.5, 4.0e-06);
		EXPECT_FALSE(movingTest.collideSegmentSegmentGeneralCase(statusFalse, 0.0, 0.75 + 1.0, &t, &r, &s, 0));
	}
};

TEST(SegmentSegmentCcdMovingContactTests, TestSegmentSegmentCoplanarCase)
{
	MockSegmentSegmentCcdMovingContact movingTest;

	std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, 1.75, 0.0), Math::Vector3d(1.0, 1.75, 0.0)};
	std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(1.0, -0.25, 0.0), Math::Vector3d(3.0, -0.25, 0.0)};
	std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(-1.0, 0.0, 0.0), Math::Vector3d(-1.0, -1.0, 0.0)};
	std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(1.0, 0.0, 0.0), Math::Vector3d(1.0, -1.0, 0.0)};

	double t;
	double r;
	double s;

	{
		// Test where collision occurs at the end of the (small) time interval
		SCOPED_TRACE("Recursion bottomed out, test collision and no collision at time precision.");

		EXPECT_TRUE(movingTest.collideSegmentSegmentCoplanarCase(
						pStart, pEnd,
						qStart, qEnd,
						0.5 - 1.4e-07, 0.5 + 1.5e-07, 3.0e-06,
						0.5, 0.25, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.5 - t));
		EXPECT_DOUBLE_EQ(0.0, r);
		EXPECT_DOUBLE_EQ(0.0, s);

		// Increase the interval and time precision so that no collision occurs
		EXPECT_FALSE(movingTest.collideSegmentSegmentCoplanarCase(
						 pEnd, pStart,
						 qStart, qEnd,
						 0.5 - .24, 0.5 + .25, .50,
						 0.5, 0.25, &t, &r, &s, 0));
	}
	{
		// Test complete recursion
		SCOPED_TRACE("Full recursion test, test start, middle, and end of interval.");

		EXPECT_TRUE(movingTest.collideSegmentSegmentCoplanarCase(
						pStart, pEnd,
						qStart, qEnd,
						0.5 - 1.5e-06, 1.0, 3.0e-06,
						0.5, 0.25, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.5 - t));
		EXPECT_DOUBLE_EQ(0.0, r);
		EXPECT_DOUBLE_EQ(0.0, s);

		EXPECT_TRUE(movingTest.collideSegmentSegmentCoplanarCase(
						pStart, pEnd,
						qStart, qEnd,
						0.0, 1.0, 3.0e-06,
						0.5, 0.25, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.5 - t));
		EXPECT_DOUBLE_EQ(0.0, r);
		EXPECT_DOUBLE_EQ(0.0, s);

		EXPECT_TRUE(movingTest.collideSegmentSegmentCoplanarCase(
						pStart, pEnd,
						qStart, qEnd,
						0.0, 0.5 + 1.5e-06, 3.0e-06,
						0.5, 0.25, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.5 - t));
		EXPECT_DOUBLE_EQ(0.0, r);
		EXPECT_DOUBLE_EQ(0.0, s);
	}
	{
		SCOPED_TRACE("Test normal flip at start, middle, and end of interval.");

		std::array<Math::Vector3d, 2> pFlipStart = {Math::Vector3d(0.0625, 1.0, 0.0),
													Math::Vector3d(-3.5 + 0.0625, -1.0e-03, 0.0)
												   };
		std::array<Math::Vector3d, 2> pFlipEnd = {Math::Vector3d(0.0625, 1.0, 0.0),
												  Math::Vector3d(3.5, -1.0e-03, 0.0)
												 };
		std::array<Math::Vector3d, 2> qFlip = {Math::Vector3d(0.0625, -1.0, 0.0),
											   Math::Vector3d(0.0625, 0.0, 0.0)
											  };

		EXPECT_TRUE(movingTest.collideSegmentSegmentCoplanarCase(
						pFlipStart, pFlipEnd,
						qFlip, qFlip,
						0.503, 1.0, 1.0e-10,
						1.0e-09, 1.0e-09, &t, &r, &s, 0));
		EXPECT_GE(1.0e-06, std::abs(0.504504 - t));
		EXPECT_GT(1.0e-03, 1.0 - r);
		EXPECT_GT(1.0e-03, 1.0 - s);

		EXPECT_TRUE(movingTest.collideSegmentSegmentCoplanarCase(
						pFlipStart, pFlipEnd,
						qFlip, qFlip,
						0.0, 1.0, 1.0e-10,
						1.0e-09, 1.0e-09, &t, &r, &s, 0));
		EXPECT_GE(1.0e-06, std::abs(0.504504 - t));
		EXPECT_GT(1.0e-03, 1.0 - r);
		EXPECT_GT(1.0e-03, 1.0 - s);

		EXPECT_TRUE(movingTest.collideSegmentSegmentCoplanarCase(
						pFlipStart, pFlipEnd,
						qFlip, qFlip,
						0.0, 0.505, 1.0e-10,
						1.0e-09, 1.0e-09, &t, &r, &s, 0));
		EXPECT_GE(1.0e-06, std::abs(0.504504 - t));
		EXPECT_GT(1.0e-03, 1.0 - r);
		EXPECT_GT(1.0e-03, 1.0 - s);
	}
	{
		SCOPED_TRACE("Test the no flip combinations.");
		std::array<Math::Vector3d, 2> pSpinStart = {Math::Vector3d(-0.5 + 1.0e-03, -1.0e-09, 0.0),
													Math::Vector3d(0.0 + 1.0e-03, 0.5, 0.0)
												   };
		std::array<Math::Vector3d, 2> pSpinEnd = {Math::Vector3d(0.5 + 1.0e-03, -1.0e-09, 0.0),
												  Math::Vector3d(1.0 + 1.0e-03, -0.5, 0.0)
												 };
		std::array<Math::Vector3d, 2> qSpin = {Math::Vector3d(0.0, -1.0, 0.0),
											   Math::Vector3d(0.0, 0.0, 0.0)
											  };

		// We don't need to test the endpoints of the region. We verified that
		// we hit those with the flip tests. Instead, test all possible combinations
		// of crossing ...
		EXPECT_TRUE(movingTest.collideSegmentSegmentCoplanarCase(
						pSpinStart, pSpinEnd,
						qSpin, qSpin,
						0.0, 1.0, 1.0e-10,
						1.0e-09, 1.0e-09, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.5 - t - 1.0e-03));
		EXPECT_GT(1.0e-08, std::abs(3.0e-06 - r));
		EXPECT_GT(1.0e-08, std::abs(1.0 - s));

		EXPECT_TRUE(movingTest.collideSegmentSegmentCoplanarCase(
						pSpinEnd, pSpinStart,
						qSpin, qSpin,
						0.0, 1.0, 1.0e-10,
						1.0e-09, 1.0e-09, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.5 - t + 1.0e-03));
		EXPECT_GT(1.0e-08, std::abs(r));
		EXPECT_GT(1.0e-08, std::abs(1.0 - s));

		EXPECT_TRUE(movingTest.collideSegmentSegmentCoplanarCase(
						qSpin, qSpin,
						pSpinStart, pSpinEnd,
						0.0, 1.0, 1.0e-10,
						1.0e-09, 1.0e-09, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.5 - t - 1.0e-03));
		EXPECT_GT(1.0e-08, std::abs(1.0 - r));
		EXPECT_GT(1.0e-08, std::abs(3.0e-06 - s));

		EXPECT_TRUE(movingTest.collideSegmentSegmentCoplanarCase(
						qSpin, qSpin,
						pSpinEnd, pSpinStart,
						0.0, 1.0, 1.0e-10,
						1.0e-09, 1.0e-09, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.5 - t + 1.0e-03));
		EXPECT_GT(1.0e-08, std::abs(1.0 - r));
		EXPECT_GT(1.0e-08, s);
	}
};

TEST(SegmentSegmentCcdMovingContactTests, TestSegmentSegmentParallelCase)
{
	MockSegmentSegmentCcdMovingContact movingTest;

	std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-5.0, -1.0e-09, 0.0),
											Math::Vector3d(-5.0, 1.0, 0.0)
										   };
	std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(5.0, -1.0e-09, 0.0),
										  Math::Vector3d(5.0, 1.0, 0.0)
										 };
	std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(0.0, -1.0, 0.0),
											Math::Vector3d(0.0, 0.0, 0.0)
										   };
	std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(0.0, -1.0, 0.0),
										  Math::Vector3d(0.0, 0.0, 0.0)
										 };

	double t;
	double r;
	double s;

	{
		// Test where collision occurs in one of the intervals
		SCOPED_TRACE("Test success and failure at end of recursion.");

		EXPECT_TRUE(movingTest.collideSegmentSegmentParallelCase(
						pStart, pEnd,
						qStart, qEnd,
						0.425 - 1.4e-07, 0.425 + 1.4e-07,
						0.5, 0.25, 3.0e-06, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.425 - t));
		EXPECT_GE(1.0e-09, std::abs(5.0e-10 - r));
		EXPECT_GE(1.0e-09, std::abs(1.0 - 5.0e-10 - s));

		EXPECT_FALSE(movingTest.collideSegmentSegmentParallelCase(
						 pStart, pEnd,
						 qStart, qEnd,
						 0.4 - 1.4e-07, 0.4 + 1.4e-07,
						 0.5, 0.25, 3.0e-06, &t, &r, &s, 0));
	}
	{
		// Test where collision occurs in one of the intervals
		SCOPED_TRACE("Test collision at end of one of the intervals, first, middle, and last.");

		EXPECT_TRUE(movingTest.collideSegmentSegmentParallelCase(
						pStart, pEnd,
						qStart, qEnd,
						0.4, 1.0,
						0.5, 0.25, 3.0e-06, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.425 - t));
		EXPECT_GE(1.0e-09, std::abs(5.0e-10 - r));
		EXPECT_GE(1.0e-09, std::abs(1.0 - 5.0e-10 - s));

		EXPECT_TRUE(movingTest.collideSegmentSegmentParallelCase(
						pStart, pEnd,
						qStart, qEnd,
						0.1, 1.0,
						0.5, 0.25, 3.0e-06, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.425 - t));
		EXPECT_GE(1.0e-09, std::abs(5.0e-10 - r));
		EXPECT_GE(1.0e-09, std::abs(1.0 - 5.0e-10 - s));

		EXPECT_TRUE(movingTest.collideSegmentSegmentParallelCase(
						pStart, pEnd,
						qStart, qEnd,
						0.1, 0.43,
						0.5, 0.25, 3.0e-06, &t, &r, &s, 0));
		EXPECT_GE(3.0e-06, std::abs(0.425 - t));
		EXPECT_GE(1.0e-09, std::abs(5.0e-10 - r));
		EXPECT_GE(1.0e-09, std::abs(1.0 - 5.0e-10 - s));
	}
	{
		// Test where collision occurs in one of the intervals
		SCOPED_TRACE("Test within one of the intervals, first, middle, and last.");

		EXPECT_TRUE(movingTest.collideSegmentSegmentParallelCase(
						pStart, pEnd,
						qStart, qEnd,
						0.42, 1.0,
						1.0e-03, 1.0e-03, 1.0e-06, &t, &r, &s, 0));
		EXPECT_GE(2.0e-06, std::abs(0.5 - 2.0e-04 - t));
		EXPECT_GE(1.0e-09, std::abs(5.0e-10 - r));
		EXPECT_GE(1.0e-09, std::abs(1.0 - 5.0e-10 - s));

		EXPECT_TRUE(movingTest.collideSegmentSegmentParallelCase(
						pStart, pEnd,
						qStart, qEnd,
						0.0, 1.0,
						1.0e-03, 1.0e-03, 1.0e-06, &t, &r, &s, 0));
		EXPECT_GE(2.0e-06, std::abs(0.5 - 2.0e-04 - t));
		EXPECT_GE(1.0e-09, std::abs(5.0e-10 - r));
		EXPECT_GE(1.0e-09, std::abs(1.0 - 5.0e-10 - s));

		EXPECT_TRUE(movingTest.collideSegmentSegmentParallelCase(
						pStart, pEnd,
						qStart, qEnd,
						0.0, 0.51,
						1.0e-03, 1.0e-03, 1.0e-06, &t, &r, &s, 0));
		EXPECT_GE(2.0e-06, std::abs(0.5 - 2.0e-04 - t));
		EXPECT_GE(1.0e-09, std::abs(5.0e-10 - r));
		EXPECT_GE(1.0e-09, std::abs(1.0 - 5.0e-10 - s));
	}
};

TEST(SegmentSegmentCcdMovingContactTests, TestRouting)
{
	// Test the base case for handling traffic to each of the 3 specific cases.
	MockSegmentSegmentCcdMovingContact movingTest;
	double r;
	double s;
	double t;

	{
		SCOPED_TRACE("Test the routing of the Parallel case.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-5.0, -1.0e-09, 0.0),
												Math::Vector3d(-5.0, 1.0, 0.0)
											   };
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(5.0, -1.0e-09, 0.0),
											  Math::Vector3d(5.0, 1.0, 0.0)
											 };
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(0.0, -1.0, 0.0),
												Math::Vector3d(0.0, 0.0, 0.0)
											   };
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(0.0, -1.0, 0.0),
											  Math::Vector3d(0.0, 0.0, 0.0)
											 };

		EXPECT_TRUE(movingTest.collideSegmentSegmentBaseCase(
						pStart, pEnd,
						qStart, qEnd,
						0.5, 0.25, 3.0e-06, &t, &r, &s));
		EXPECT_GE(3.0e-06, std::abs(0.425 - t));
		EXPECT_GE(1.0e-09, std::abs(5.0e-10 - r));
		EXPECT_GE(1.0e-09, std::abs(1.0 - 5.0e-10 - s));
	}
	{
		SCOPED_TRACE("Test the routing of the Coplanar case.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-0.5 + 1.0e-03, -1.0e-09, 0.0),
												Math::Vector3d(0.0 + 1.0e-03, 0.5, 0.0)
											   };
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(0.5 + 1.0e-03, -1.0e-09, 0.0),
											  Math::Vector3d(1.0 + 1.0e-03, -0.5, 0.0)
											 };
		std::array<Math::Vector3d, 2> q = {Math::Vector3d(0.0, -1.0, 0.0),
										   Math::Vector3d(0.0, 0.0, 0.0)
										  };

		EXPECT_TRUE(movingTest.collideSegmentSegmentBaseCase(pStart, pEnd, q, q,
					1.0e-09, 1.0e-09, 1.0e-10, &t, &r, &s));
		EXPECT_GE(3.0e-06, std::abs(0.5 - t - 1.0e-03));
		EXPECT_GT(1.0e-08, std::abs(3.0e-06 - r));
		EXPECT_GT(1.0e-08, std::abs(1.0 - s));
	}
	{
		SCOPED_TRACE("Test the routing of the General case.");
		std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-1.0, -1.0, -2.0), Math::Vector3d(1.0, 1.0, -2.0)};
		std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(-1.0, -1.0, 0.0), Math::Vector3d(1.0, 1.0, 0.0)};
		std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(1.0, -1.0, 1.0), Math::Vector3d(-1.0, 1.0, 1.0)};
		std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(1.0, -1.0, 0.0), Math::Vector3d(-1.0, 1.0, 0.0)};

		EXPECT_TRUE(movingTest.collideSegmentSegmentBaseCase(pStart, pEnd, qStart, qEnd,
					0.5, 0.25, 3.0e-06, &t, &r, &s));
		EXPECT_GT(epsilon, std::abs(0.75 - t));
		EXPECT_DOUBLE_EQ(0.50, r);
		EXPECT_DOUBLE_EQ(0.50, s);
	}
};

TEST(SegmentSegmentCcdMovingContactTests, PublicAPI)
{
	// Test the base case for handling traffic to each of the 3 specific cases.
	MockSegmentSegmentCcdMovingContact movingTest;

	std::array<Math::Vector3d, 2> pStart = {Math::Vector3d(-5.0, -1.0e-09, 0.0),
											Math::Vector3d(-5.0, 1.0, 0.0)
										   };
	std::array<Math::Vector3d, 2> pEnd = {Math::Vector3d(5.0, -1.0e-09, 0.0),
										  Math::Vector3d(5.0, 1.0, 0.0)
										 };
	std::array<Math::Vector3d, 2> qStart = {Math::Vector3d(0.0, -1.0, 0.0),
											Math::Vector3d(0.0, 0.0, 0.0)
										   };
	std::array<Math::Vector3d, 2> qEnd = {Math::Vector3d(0.0, -1.0, 0.0),
										  Math::Vector3d(0.0, 0.0, 0.0)
										 };
	Math::Vector3d dir;

	double r;
	double s;
	double t;

	{
		SCOPED_TRACE("Test thick segments.");

		EXPECT_TRUE(movingTest.collideMovingSegmentSegment(
						pStart, pEnd,
						qStart, qEnd,
						0.5, 0.25, 3.0e-06, &t, &r, &s, &dir));
		EXPECT_GE(3.0e-06, std::abs(0.425 - t));
		EXPECT_GE(1.0e-09, std::abs(5.0e-10 - r));
		EXPECT_GE(1.0e-09, std::abs(1.0 - 5.0e-10 - s));
		EXPECT_TRUE(dir.normalized().isApprox(Math::Vector3d(1.0, 0.0, 0.0), 1.0e-09));
	}
	{
		SCOPED_TRACE("Test zero thickness segments.");

		EXPECT_TRUE(movingTest.collideMovingSegmentSegment(
						pStart, pEnd,
						qStart, qEnd,
						2.0e-03, 1.0e-06, &t, &r, &s, &dir));
		EXPECT_GE(2.0e-06, std::abs(0.5 - 2.0e-04 - t));
		EXPECT_GE(1.0e-09, std::abs(5.0e-10 - r));
		EXPECT_GE(1.0e-09, std::abs(1.0 - 5.0e-10 - s));
		EXPECT_TRUE(dir.normalized().isApprox(Math::Vector3d(1.0, 0.0, 0.0), 1.0e-09));
	}
};

}; // namespace Collision
}; // namespace SurgSim

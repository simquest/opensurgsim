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

#include "SurgSim/Collision/SegmentSegmentCcdStaticContact.h"

namespace SurgSim
{

namespace Collision
{

namespace
{
double epsilon = 1.0e-10;
}

template <typename T>
class SegmentSegmentCcdStaticContactTests : public ::testing::Test
{
};

class MockSegmentSegmentCcdStaticContact : public SurgSim::Collision::SegmentSegmentCcdStaticContact
{
public:
	bool collideStaticPointSegment(
		const Math::Vector3d& point, const std::array<SurgSim::Math::Vector3d, 2>& p,
		double thicknessA, double thicknessP,
		double* r)
	{
		return SegmentSegmentCcdStaticContact::collideStaticPointSegment(point, p, thicknessA, thicknessP, r);
	}

	int computeCollisionRegion(double r, double s, double ratio) const
	{
		return SegmentSegmentCcdStaticContact::computeCollisionRegion(r, s, ratio);
	}

	SegmentCcdEdgeType computeCollisionEdge(int region, double a, double b, double d) const
	{
		return SegmentSegmentCcdStaticContact::computeCollisionEdge(region, a, b, d);
	}

	void computeCollisionParametrics(SegmentCcdEdgeType edge, double a, double b, double c, double d, double e,
									 double ratio, double* r, double* s) const
	{
		SegmentSegmentCcdStaticContact::computeCollisionParametrics(edge, a, b, c, d, e, ratio, r, s);
	}

	void computeParallelSegmentParametrics(double a, double b, double d, double* r, double* s)
	{
		SegmentSegmentCcdStaticContact::computeParallelSegmentParametrics(a, b, d, r, s);
	}
};

TEST(SegmentSegmentCcdStaticContactTests, Initialization)
{
	ASSERT_NO_THROW(SegmentSegmentCcdStaticContact staticTest);
	SegmentSegmentCcdStaticContact staticTest;
};

TEST(SegmentSegmentCcdStaticContactTests, RegionGenerator)
{
	MockSegmentSegmentCcdStaticContact staticTest;
	const double ratio = 0.5;

	for (double r = -1.0; r <= 0.0; r += 0.1)
	{
		for (double s = -1.0; s < 0.0; s += 0.1)
		{
			EXPECT_EQ(6, staticTest.computeCollisionRegion(r, s, ratio));
		}

		for (double s = 0.0; s <= ratio; s += 0.1)
		{
			EXPECT_EQ(5, staticTest.computeCollisionRegion(r, s, ratio));
		}

		for (double s = ratio + 0.1; s <= 2 * ratio; s += 0.1)
		{
			EXPECT_EQ(4, staticTest.computeCollisionRegion(r, s, ratio));
		}
	}

	for (double r = 0.0; r <= ratio; r += 0.1)
	{
		for (double s = -1.0; s < 0.0; s += 0.1)
		{
			EXPECT_EQ(7, staticTest.computeCollisionRegion(r, s, ratio));
		}

		for (double s = 0.0; s <= ratio; s += 0.1)
		{
			EXPECT_EQ(0, staticTest.computeCollisionRegion(r, s, ratio));
		}

		for (double s = ratio + 0.1; s <= 2 * ratio; s += 0.1)
		{
			EXPECT_EQ(3, staticTest.computeCollisionRegion(r, s, ratio));
		}
	}

	for (double r = ratio + 0.1; r <= 2 * ratio; r += 0.1)
	{
		for (double s = -1.0; s < 0.0; s += 0.1)
		{
			EXPECT_EQ(8, staticTest.computeCollisionRegion(r, s, ratio));
		}

		for (double s = 0.0; s <= ratio; s += 0.1)
		{
			EXPECT_EQ(1, staticTest.computeCollisionRegion(r, s, ratio));
		}

		for (double s = ratio + 0.1; s <= 2 * ratio; s += 0.1)
		{
			EXPECT_EQ(2, staticTest.computeCollisionRegion(r, s, ratio));
		}
	}
};

TEST(SegmentSegmentCcdStaticContactTests, EdgeGenerator)
{
	MockSegmentSegmentCcdStaticContact staticTest;
	const double ratio = 0.5;
	double a = 1.0;
	double b = 1.0;
	double d = 3.0;

	// Both on segments
	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeEdgeSkip, staticTest.computeCollisionEdge(0, a, b, d));

	// One on segment, one off.
	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR1, staticTest.computeCollisionEdge(1, a, b, d));
	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS1, staticTest.computeCollisionEdge(3, a, b, d));
	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR0, staticTest.computeCollisionEdge(5, a, b, d));
	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS0, staticTest.computeCollisionEdge(7, a, b, d));

	// Both off.
	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS1, staticTest.computeCollisionEdge(2, a, b, d));
	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR1, staticTest.computeCollisionEdge(2, a, b, -d));

	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR0, staticTest.computeCollisionEdge(4, a, b, d));
	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS1, staticTest.computeCollisionEdge(4, a, b, -d));

	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR0, staticTest.computeCollisionEdge(6, a, b, d));
	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS0, staticTest.computeCollisionEdge(6, a, b, -d));

	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS0, staticTest.computeCollisionEdge(8, a, b, d));
	EXPECT_EQ(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR1, staticTest.computeCollisionEdge(8, a, b, -d));
};

TEST(SegmentSegmentCcdStaticContactTests, CalculateParametrics)
{
	MockSegmentSegmentCcdStaticContact staticTest;

	double a = 3.3;
	double b = 3.2;
	double c = 3.0;
	double d = 3.2;
	double e = -4.2;
	double ratio = 5.0;
	double r = 4.5;
	double s = 4.0;

	SCOPED_TRACE("Testing region 0. Nearest point is on both segments.");
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeEdgeSkip,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(0.9, r);
	EXPECT_DOUBLE_EQ(0.8, s);

	SCOPED_TRACE("Testing when r = 0. s depends on e and c.");
	e = 5.0;
	c = 7.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR0,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(0.0, r);
	EXPECT_DOUBLE_EQ(0.0, s);

	e = -5.0;
	c = 7.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR0,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(0.0, r);
	EXPECT_DOUBLE_EQ(5.0 / 7.0, s);

	e = -5.0;
	c = 4.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR0,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(0.0, r);
	EXPECT_DOUBLE_EQ(1.0, s);

	SCOPED_TRACE("Testing when r = 1. s depends on e, b, and c.");
	e = 5.0;
	b = -4.0;
	c = 4.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR1,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(1.0, r);
	EXPECT_DOUBLE_EQ(0.0, s);

	e = -5.0;
	b = 4.0;
	c = 4.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR1,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(1.0, r);
	EXPECT_DOUBLE_EQ(0.25, s);

	e = -5.0;
	b = -4.0;
	c = 4.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeR1,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(1.0, r);
	EXPECT_DOUBLE_EQ(1.0, s);

	SCOPED_TRACE("Testing when s = 0. r depends on d and a.");
	d = 7.0;
	a = 8.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS0,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(0.0, r);
	EXPECT_DOUBLE_EQ(0.0, s);

	d = -7.0;
	a = 8.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS0,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(0.875, r);
	EXPECT_DOUBLE_EQ(0.0, s);

	d = -9.0;
	a = 8.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS0,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(1.0, r);
	EXPECT_DOUBLE_EQ(0.0, s);

	SCOPED_TRACE("Testing when s = 1. r depends on d, b, and a.");
	d = 6.0;
	b = -1.0;
	a = 8.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS1,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(0.0, r);
	EXPECT_DOUBLE_EQ(1.0, s);

	d = -6.0;
	b = -1.0;
	a = 8.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS1,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(0.875, r);
	EXPECT_DOUBLE_EQ(1.0, s);

	d = -6.0;
	b = -6.0;
	a = 8.0;
	staticTest.computeCollisionParametrics(SegmentSegmentCcdStaticContact::SegmentCcdEdgeTypeS1,
										   a, b, c, d, e, ratio, &r, &s);
	EXPECT_DOUBLE_EQ(1.0, r);
	EXPECT_DOUBLE_EQ(1.0, s);

};

TEST(SegmentSegmentCcdStaticContactTests, CalculateParallelParametrics)
{
	MockSegmentSegmentCcdStaticContact staticTest;

	double a;
	double b;
	double d;
	double r = 4.5;
	double s = 4.0;

	SCOPED_TRACE("Segments in opposite directions. End point r = s = 0.");
	a = 4.0;
	b = 3.2;
	d = 3.2;
	staticTest.computeParallelSegmentParametrics(a, b, d, &r, &s);
	EXPECT_DOUBLE_EQ(0.0, r);
	EXPECT_DOUBLE_EQ(0.0, s);

	SCOPED_TRACE("Segments in opposite directions. End point s0 in r.");
	a = 4.0;
	b = 4.0;
	d = -3.2;
	staticTest.computeParallelSegmentParametrics(a, b, d, &r, &s);
	EXPECT_DOUBLE_EQ(0.8, r);
	EXPECT_DOUBLE_EQ(0.0, s);

	SCOPED_TRACE("Segments in opposite directions. End point r = s = 1.");
	a = 4.0;
	b = 3.2;
	d = -9.2;
	staticTest.computeParallelSegmentParametrics(a, b, d, &r, &s);
	EXPECT_DOUBLE_EQ(1.0, r);
	EXPECT_DOUBLE_EQ(1.0, s);

	SCOPED_TRACE("Segments in opposite directions. End point r = 1 is closest to s.");
	a = 4.0;
	b = 3.2;
	d = -5.6;
	staticTest.computeParallelSegmentParametrics(a, b, d, &r, &s);
	EXPECT_DOUBLE_EQ(1.0, r);
	EXPECT_DOUBLE_EQ(0.5, s);

	SCOPED_TRACE("Segments in same directions, but don't overlap. End point r = 1, s = 0.");
	a = 4.0;
	b = -3.2;
	d = -4.2;
	staticTest.computeParallelSegmentParametrics(a, b, d, &r, &s);
	EXPECT_DOUBLE_EQ(1.0, r);
	EXPECT_DOUBLE_EQ(0.0, s);

	SCOPED_TRACE("Segments in same directions, r overlaps s0.");
	a = 4.0;
	b = -3.2;
	d = -3.0;
	staticTest.computeParallelSegmentParametrics(a, b, d, &r, &s);
	EXPECT_DOUBLE_EQ(0.75, r);
	EXPECT_DOUBLE_EQ(0.0, s);

	SCOPED_TRACE("Segments in same directions, end points r0 and s1 closest.");
	a = 4.0;
	b = -3.2;
	d = 6.0;
	staticTest.computeParallelSegmentParametrics(a, b, d, &r, &s);
	EXPECT_DOUBLE_EQ(0.0, r);
	EXPECT_DOUBLE_EQ(1.0, s);

	SCOPED_TRACE("Segments in same directions, and r0 overlaps s.");
	a = 4.0;
	b = -3.2;
	d = 2.4;
	staticTest.computeParallelSegmentParametrics(a, b, d, &r, &s);
	EXPECT_DOUBLE_EQ(0.0, r);
	EXPECT_DOUBLE_EQ(0.75, s);
};

TEST(SegmentSegmentCcdStaticContactTests, PointSegmentCollisions)
{
	MockSegmentSegmentCcdStaticContact staticTest;

	Math::Vector3d point;
	Math::Vector3d p0;
	Math::Vector3d p1;
	std::array<SurgSim::Math::Vector3d, 2> p;
	double thicknessA = 0.25;
	double thicknessP = 0.75;
	double r;

	SCOPED_TRACE("Segment degenerate, end 0 is close enough.");
	point = Math::Vector3d(0.0, 0.0, 0.0);
	p0 = Math::Vector3d(0.0, 0.0, 1.0 - 1.0e-09 / 4.0);
	p1 = Math::Vector3d(0.0, 0.0, 1.0 + 1.0e-09 / 4.0);
	p[0] = p0;
	p[1] = p1;
	EXPECT_TRUE(staticTest.collideStaticPointSegment(point, p, thicknessA, thicknessP, &r));
	EXPECT_DOUBLE_EQ(0.0, r);

	SCOPED_TRACE("Segment degenerate, end 1 is close enough.");
	point = Math::Vector3d(0.0, 0.0, 0.0);
	p0 = Math::Vector3d(0.0, 0.0, 1.0 - 1.0e-09 / 4.0);
	p1 = Math::Vector3d(0.0, 0.0, 1.0 + 1.0e-09 / 4.0);
	p[0] = p1;
	p[1] = p0;
	EXPECT_TRUE(staticTest.collideStaticPointSegment(point, p, thicknessA, thicknessP, &r));
	EXPECT_DOUBLE_EQ(1.0, r);

	SCOPED_TRACE("Segment degenerate, neither point close enough.");
	point = Math::Vector3d(0.0, 0.0, 0.0);
	p0 = Math::Vector3d(0.0, 0.0, 1.0 - 1.0e-09 / 4.0);
	p1 = Math::Vector3d(1.0, 0.0, 1.0 + 1.0e-09 / 4.0);
	p[0] = p1;
	p[1] = p1;
	EXPECT_FALSE(staticTest.collideStaticPointSegment(point, p, thicknessA, thicknessP, &r));

	SCOPED_TRACE("Segment clamped to 0, close enough");
	point = Math::Vector3d(0.0, 0.5, 0.0);
	p0 = Math::Vector3d(0.5, 0.0, 0.25);
	p1 = Math::Vector3d(1.5, 0.0, 0.25);
	p[0] = p0;
	p[1] = p1;
	EXPECT_TRUE(staticTest.collideStaticPointSegment(point, p, thicknessA, thicknessP, &r));
	EXPECT_DOUBLE_EQ(0.0, r);

	SCOPED_TRACE("Segment clamped to 0, not close enough");
	point = Math::Vector3d(0.0, 1.0, 0.0);
	p0 = Math::Vector3d(0.5, 0.0, 0.25);
	p1 = Math::Vector3d(1.5, 0.0, 0.25);
	p[0] = p0;
	p[1] = p1;
	EXPECT_FALSE(staticTest.collideStaticPointSegment(point, p, thicknessA, thicknessP, &r));

	SCOPED_TRACE("Segment clamped to 1, close enough");
	point = Math::Vector3d(0.0, 0.5, 0.0);
	p0 = Math::Vector3d(0.5, 0.0, 0.25);
	p1 = Math::Vector3d(1.5, 0.0, 0.25);
	p[0] = p1;
	p[1] = p0;
	EXPECT_TRUE(staticTest.collideStaticPointSegment(point, p, thicknessA, thicknessP, &r));
	EXPECT_DOUBLE_EQ(1.0, r);

	SCOPED_TRACE("Segment clamped to 1, not close enough");
	point = Math::Vector3d(0.0, 1.0, 0.0);
	p0 = Math::Vector3d(0.5, 0.0, 0.25);
	p1 = Math::Vector3d(1.5, 0.0, 0.25);
	p[0] = p1;
	p[1] = p0;
	EXPECT_FALSE(staticTest.collideStaticPointSegment(point, p, thicknessA, thicknessP, &r));

	SCOPED_TRACE("Segment not clamped, close enough");
	point = Math::Vector3d(0.0, 0.5, 0.0);
	p0 = Math::Vector3d(-1.5, 0.0, 0.25);
	p1 = Math::Vector3d(1.5, 0.0, 0.25);
	p[0] = p0;
	p[1] = p1;
	EXPECT_TRUE(staticTest.collideStaticPointSegment(point, p, thicknessA, thicknessP, &r));
	EXPECT_DOUBLE_EQ(0.5, r);

	SCOPED_TRACE("Segment not clamped, not close enough");
	point = Math::Vector3d(0.0, 1.0, 0.0);
	p0 = Math::Vector3d(0.5, 0.0, 0.25);
	p1 = Math::Vector3d(1.5, 0.0, 0.25);
	p[0] = p0;
	p[1] = p1;
	EXPECT_FALSE(staticTest.collideStaticPointSegment(point, p, thicknessA, thicknessP, &r));
};

TEST(SegmentSegmentCcdStaticContactTests, SegmentSegmentCollisions)
{
	MockSegmentSegmentCcdStaticContact staticTest;

	Math::Vector3d p0;
	Math::Vector3d p1;
	std::array<SurgSim::Math::Vector3d, 2> p;
	Math::Vector3d q0;
	Math::Vector3d q1;
	std::array<SurgSim::Math::Vector3d, 2> q;
	double radiusP = 0.25;
	double radiusQ = 0.75;
	double distanceEpsilon = radiusP + radiusQ;
	double r;
	double s;
	Math::Vector3d increment(0.1, 0, 0);

	{
		SCOPED_TRACE("Test degenerate segment cases - 2 points");
		p0 = Math::Vector3d(-1.0e-09 / 4.0, 0.0, 1.0);
		p1 = Math::Vector3d(1.0e-09 / 4.0, 0.0, 1.0);
		q0 = Math::Vector3d(-1.0 - 1.0e-09 / 4.0, 0.0, 0.5);
		q1 = Math::Vector3d(-1.0 + 1.0e-09 / 4.0, 0.0, 0.5);

		for (size_t ctr = 0; ctr < 2; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 17; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 2; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			q0 += increment;
			q1 += increment;
		}

	}

	{
		SCOPED_TRACE("Test degenerate segment cases - one segment is point");
		p0 = Math::Vector3d(0.0, 0.0, 1.0);
		p1 = Math::Vector3d(1.0e-09 / 2.0, 0.0, 1.0);
		q0 = Math::Vector3d(-1.0, 0.0, 0.5);
		q1 = Math::Vector3d(-2.0, 0.0, 0.5);

		for (size_t ctr = 0; ctr < 2; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 8; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 10; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_NEAR(ctr / 10.0, s, epsilon);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_NEAR(ctr / 10.0, s, epsilon);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_NEAR(ctr / 10.0, r, epsilon);
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			EXPECT_NEAR(ctr / 10.0, r, epsilon);
			EXPECT_DOUBLE_EQ(0.0, s);
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 9; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, r);
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, r);
			EXPECT_DOUBLE_EQ(0.0, s);
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 2; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			q0 += increment;
			q1 += increment;
		}
	}

	{
		SCOPED_TRACE("Test degenerate segment cases - Parallel segments");
		Math::Vector3d newP;
		Math::Vector3d newQ;

		p0 = Math::Vector3d(0.0, 0.0, 1.0);
		p1 = Math::Vector3d(1.0, 0.0, 1.0);
		q0 = Math::Vector3d(-1.0, 0.0, 0.5);
		q1 = Math::Vector3d(-2.0, 0.0, 0.5);

		for (size_t ctr = 0; ctr < 2; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 8; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 20; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			newP = Math::interpolate(p0, p1, r);
			newQ = Math::interpolate(q0, q1, s);
			EXPECT_NEAR(0.5, (newP - newQ).norm(), epsilon);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			newP = Math::interpolate(p0, p1, r);
			newQ = Math::interpolate(q0, q1, s);
			EXPECT_NEAR(0.5, (newP - newQ).norm(), epsilon);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			newP = Math::interpolate(p0, p1, s);
			newQ = Math::interpolate(q0, q1, r);
			EXPECT_NEAR(0.5, (newP - newQ).norm(), epsilon);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			newP = Math::interpolate(p0, p1, s);
			newQ = Math::interpolate(q0, q1, r);
			EXPECT_NEAR(0.5, (newP - newQ).norm(), epsilon);
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 9; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, s);
			EXPECT_DOUBLE_EQ(1.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, s);
			EXPECT_DOUBLE_EQ(1.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, r);
			EXPECT_DOUBLE_EQ(1.0, s);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, r);
			EXPECT_DOUBLE_EQ(1.0, s);
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 2; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			q0 += increment;
			q1 += increment;
		}
	}

	{
		SCOPED_TRACE("Test intersecting segments");
		Math::Vector3d newP;
		Math::Vector3d newQ;

		p0 = Math::Vector3d(0.0, -1.0, 1.0);
		p1 = Math::Vector3d(0.0, 1.0, 1.0);
		q0 = Math::Vector3d(-1.0, 0.0, 0.5);
		q1 = Math::Vector3d(-3.0, 0.0, 0.5);

		for (size_t ctr = 0; ctr < 2; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 8; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.5, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(0.0, s);
			EXPECT_DOUBLE_EQ(0.5, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(0.5, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(0.5, s);
			EXPECT_DOUBLE_EQ(0.0, r);
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 20; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			newP = Math::interpolate(p0, p1, r);
			newQ = Math::interpolate(q0, q1, s);
			EXPECT_NEAR(0.5, (newP - newQ).norm(), epsilon);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			newP = Math::interpolate(p0, p1, r);
			newQ = Math::interpolate(q0, q1, s);
			EXPECT_NEAR(0.5, (newP - newQ).norm(), epsilon);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			newP = Math::interpolate(p0, p1, s);
			newQ = Math::interpolate(q0, q1, r);
			EXPECT_NEAR(0.5, (newP - newQ).norm(), epsilon);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			newP = Math::interpolate(p0, p1, s);
			newQ = Math::interpolate(q0, q1, r);
			EXPECT_NEAR(0.5, (newP - newQ).norm(), epsilon);
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 9; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, s);
			EXPECT_DOUBLE_EQ(0.5, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, s);
			EXPECT_DOUBLE_EQ(0.5, r);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, r);
			EXPECT_DOUBLE_EQ(0.5, s);
			EXPECT_TRUE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			EXPECT_DOUBLE_EQ(1.0, r);
			EXPECT_DOUBLE_EQ(0.5, s);
			q0 += increment;
			q1 += increment;
		}

		for (size_t ctr = 0; ctr < 2; ++ctr)
		{
			p[0] = p0;
			p[1] = p1;
			q[0] = q0;
			q[1] = q1;
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(p, q, radiusP, radiusQ, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, distanceEpsilon, &r, &s));
			EXPECT_FALSE(staticTest.collideStaticSegmentSegment(q, p, radiusP, radiusQ, &r, &s));
			q0 += increment;
			q1 += increment;
		}
	}
};

}; // namespace Collision
}; // namespace SurgSim

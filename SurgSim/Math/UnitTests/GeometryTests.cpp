// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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
/// Tests for the Geometry.cpp functions.
///


#include <gtest/gtest.h>
#include <array>
#include <numeric>
#include <cmath>

#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/UnitTests/MockTriangle.h"
#include <boost/math/special_functions/fpclassify.hpp>

namespace SurgSim
{
namespace Math
{

typedef double SizeType;
typedef Eigen::Matrix<SizeType, 3, 1> VectorType;

::std::ostream& operator <<(std::ostream& stream, const VectorType& vector)
{
	stream << "(" << vector[0] << ", " << vector[1] << ", " << vector[2] << ")";
	return stream;
}

bool near(double val1, double val2, double abs_error)
{
	const double diff = std::abs(val1 - val2);
	if (diff <= abs_error)
	{
		return true;
	}
	else
	{
		return false;
	}
}

::testing::AssertionResult eigenEqual(const VectorType& expected, const VectorType& actual)
{
	double precision = 1e-4;
	if (expected.isApprox(actual, precision))
	{
		return ::testing::AssertionSuccess();
	}
	else
	{
		return ::testing::AssertionFailure() << "Eigen Matrices not the same " << std::endl <<
			   "expected: " << expected << std::endl << "actual: " << actual << std::endl;
	}
}

::testing::AssertionResult eigenAllNan(const VectorType& actual)
{
	if (boost::math::isnan(actual[0]) && boost::math::isnan(actual[1]) && boost::math::isnan(actual[2]))
	{
		return ::testing::AssertionSuccess();
	}
	else
	{
		return ::testing::AssertionFailure() << "Not all elements are NAN";
	}
}

class Segment
{
public:
	VectorType a;
	VectorType b;
	VectorType ab;

	Segment() {}
	Segment(const VectorType& pointA, const VectorType& pointB) :
		a(pointA), b(pointB), ab(pointB - pointA) {}
	~Segment() {}
	/// Point on the line that the segment is on, s =< 1 and s >= 0 will give you a point on the segment
	VectorType pointOnLine(double s) const
	{
		return a + ab * s;
	}
};


namespace
{
SizeType epsilon = 1e-10;
}

class GeometryTest : public ::testing::Test
{
protected:
	virtual void SetUp()
	{
		plainPoint = VectorType(45, 20, 10);
		plainSegment = Segment(VectorType(1.1, 2.2, 3.3), VectorType(6.6, 5.5, 4.4));
		plainNormal = plainSegment.ab.cross(plainPoint); // Normal to segment
		plainNormal.normalize();

		degenerateSegment.a = plainSegment.a;
		degenerateSegment.b = degenerateSegment.a + (plainSegment.ab) * 1e-9;
		degenerateSegment.ab = degenerateSegment.b - degenerateSegment.a;

		plainLine = Segment(VectorType(-10.0, 10, 10), VectorType(10.0, 10.0, 10.0));
		parallelLine = Segment(VectorType(-100.0, 5.0, 5.0), VectorType(-90.0, 5.0, 5.0));
		intersectingLine = Segment(VectorType(0, 0, 0), VectorType(20, 20, 20));
		nonIntersectingLine = Segment(VectorType(5, 5, -5), VectorType(5, 5, 5));

		tri = MockTriangle(VectorType(5, 0, 0), VectorType(0, -5, -5), VectorType(0, 5, 5));
	}

	virtual void TearDown()
	{
	}
	VectorType plainPoint;
	Vector3d plainNormal;
	Segment plainSegment;
	Segment degenerateSegment;

	Segment plainLine;
	Segment parallelLine;
	Segment intersectingLine;
	Segment nonIntersectingLine;

	MockTriangle tri;
};


TEST_F(GeometryTest, BaryCentricWithNormal)
{
	// Order of Points is v0,v1,v2
	//Check Edges first
	VectorType outputPoint;
	EXPECT_TRUE(barycentricCoordinates(tri.v0, tri.v0, tri.v1, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(1, 0, 0), outputPoint));

	EXPECT_TRUE(barycentricCoordinates(tri.v1, tri.v0, tri.v1, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0, 1, 0), outputPoint));

	EXPECT_TRUE(barycentricCoordinates(tri.v2, tri.v0, tri.v1, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0, 0, 1), outputPoint));

	// Halfway points
	EXPECT_TRUE(barycentricCoordinates<double>(tri.pointInTriangle(0.5, 0),
				tri.v0, tri.v1, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.5, 0.5, 0), outputPoint));

	EXPECT_TRUE(barycentricCoordinates<double>(tri.pointInTriangle(0, 0.5),
				tri.v0, tri.v1, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.5, 0.0, 0.5), outputPoint));

	// Center Point
	VectorType inputPoint;
	inputPoint = (tri.v0 + tri.v1 + tri.v2) / 3;
	EXPECT_TRUE(barycentricCoordinates(inputPoint, tri.v0, tri.v1, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0), outputPoint));

	// random Point
	inputPoint = tri.v0 * 0.2 + tri.v1 * 0.25 + tri.v2 * 0.55;
	EXPECT_TRUE(barycentricCoordinates(inputPoint, tri.v0, tri.v1, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.2, 0.25, 0.55), outputPoint));

	// Degenerate
	EXPECT_FALSE(barycentricCoordinates(inputPoint, tri.v1, tri.v1, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));

	EXPECT_FALSE(barycentricCoordinates(inputPoint, tri.v0, tri.v0, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));

	EXPECT_FALSE(barycentricCoordinates(inputPoint, tri.v2, tri.v1, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));
}

TEST_F(GeometryTest, BaryCentricWithoutNormal)
{
	// Order of Points is v0,v1,v2
	//Check Edges first
	VectorType outputPoint;
	EXPECT_TRUE(barycentricCoordinates(tri.v0, tri.v0, tri.v1, tri.v2, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(1, 0, 0), outputPoint));

	EXPECT_TRUE(barycentricCoordinates(tri.v1, tri.v0, tri.v1, tri.v2, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0, 1, 0), outputPoint));

	EXPECT_TRUE(barycentricCoordinates(tri.v2, tri.v0, tri.v1, tri.v2, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0, 0, 1), outputPoint));

	// Halfway points
	EXPECT_TRUE(barycentricCoordinates<double>(tri.pointInTriangle(0.5, 0),
				tri.v0, tri.v1, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.5, 0.5, 0), outputPoint));

	EXPECT_TRUE(barycentricCoordinates<double>(tri.pointInTriangle(0, 0.5),
				tri.v0, tri.v1, tri.v2, tri.n, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.5, 0.0, 0.5), outputPoint));

	// Center Point
	VectorType inputPoint;
	inputPoint = (tri.v0 + tri.v1 + tri.v2) / 3;
	EXPECT_TRUE(barycentricCoordinates(inputPoint, tri.v0, tri.v1, tri.v2, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0), outputPoint));

	// random Point
	inputPoint = tri.v0 * 0.2 + tri.v1 * 0.25 + tri.v2 * 0.55;
	EXPECT_TRUE(barycentricCoordinates(inputPoint, tri.v0, tri.v1, tri.v2, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.2, 0.25, 0.55), outputPoint));

	// Degenerate
	EXPECT_FALSE(barycentricCoordinates(inputPoint, tri.v1, tri.v1, tri.v2, &outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));

	EXPECT_FALSE(barycentricCoordinates(inputPoint, tri.v0, tri.v0, tri.v2, &outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));

	EXPECT_FALSE(barycentricCoordinates(inputPoint, tri.v2, tri.v1, tri.v2, &outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));
}

TEST_F(GeometryTest, DistancePointLine)
{
	SizeType distance;
	Vector3d result;

	// Trivial point lies on the line
	distance = distancePointLine<SizeType>(plainSegment.pointOnLine(0.5), plainSegment.a, plainSegment.b, &result);
	EXPECT_NEAR(0.0, distance, epsilon);
	EXPECT_EQ(plainSegment.pointOnLine(0.5), result);

	// Point is away from the line
	Vector3d offLinePoint = plainSegment.a + (plainNormal * 1.5);
	distance = distancePointLine(offLinePoint, plainSegment.a, plainSegment.b, &result);
	EXPECT_NEAR(1.5, distance, epsilon);
	EXPECT_EQ(plainSegment.a, result);

	// Degenerate line, just do plain distance
	offLinePoint = plainSegment.a + plainNormal * 1.5;
	distance = distancePointLine(offLinePoint, plainSegment.a, plainSegment.a, &result);
	EXPECT_NEAR(1.5, distance, epsilon);
	EXPECT_EQ(plainSegment.a, result);
}

TEST_F(GeometryTest, DistancePointSegment)
{
	SizeType distance;
	Vector3d result;

	// Trivial point lies on the line
	distance = distancePointSegment(plainSegment.pointOnLine(0.5), plainSegment.a, plainSegment.b, &result);
	EXPECT_NEAR(0.0, distance, epsilon);
	EXPECT_EQ(plainSegment.pointOnLine(0.5), result);

	// Point On the line but outside the segment
	VectorType point = plainSegment.pointOnLine(1.5);
	distance = distancePointSegment(point, plainSegment.a, plainSegment.b, &result);
	EXPECT_NEAR((plainSegment.ab.norm() * 0.5), distance, epsilon);
	EXPECT_TRUE(eigenEqual(plainSegment.b, result));

	// Point projection is on the segment
	VectorType resultPoint = plainSegment.a + plainSegment.ab * 0.25;
	VectorType offLinePoint = resultPoint + (plainNormal * 1.5);
	distance = distancePointSegment(offLinePoint, plainSegment.a, plainSegment.b, &result);
	EXPECT_NEAR(1.5, distance, epsilon);
	EXPECT_TRUE(eigenEqual(resultPoint, result));

	// Point projection is away from the segment, distance is to the closest segment point
	resultPoint = plainSegment.a;
	offLinePoint = plainSegment.a - plainSegment.ab + (plainNormal * 1.5);
	distance = distancePointSegment(offLinePoint, plainSegment.a, plainSegment.b, &result);
	EXPECT_NEAR((offLinePoint - resultPoint).norm(), distance, epsilon);
	EXPECT_TRUE(eigenEqual(resultPoint, result));

	// Other Side of the above case
	resultPoint = plainSegment.b;
	offLinePoint = plainSegment.b + plainSegment.ab * 0.01 + plainNormal * 0.1;
	distance = distancePointSegment(offLinePoint, plainSegment.a, plainSegment.b, &result);
	EXPECT_NEAR((offLinePoint - resultPoint).norm(), distance, epsilon);
	EXPECT_TRUE(eigenEqual(resultPoint, result));


	// Degenerated Segment
	distance = distancePointSegment(offLinePoint, plainSegment.a, plainSegment.a, &result);
	EXPECT_NEAR((offLinePoint - plainSegment.a).norm(), distance, epsilon);
	EXPECT_TRUE(eigenEqual(plainSegment.a, result));
}

typedef std::tuple<Segment, Segment, VectorType, VectorType> LineLineCheckData;
void checkLineLineDistance(const LineLineCheckData& data)
{
	Segment line0 = std::get<0>(data);
	Segment line1 = std::get<1>(data);
	VectorType expectedResult0 = std::get<2>(data);
	VectorType expectedResult1 = std::get<3>(data);
	VectorType result0, result1;
	double distance;

	{
		SCOPED_TRACE("Forward Case");
		distance = distanceLineLine(line0.a, line0.b, line1.a, line1.b, &result0, &result1);
		EXPECT_NEAR((expectedResult1 - expectedResult0).norm(), distance, epsilon);
		EXPECT_TRUE(expectedResult0.isApprox(result0));
		EXPECT_TRUE(expectedResult1.isApprox(result1));
	}

	{
		SCOPED_TRACE("Backward Case");
		distance = distanceLineLine(line1.a, line1.b, line0.a, line0.b, &result1, &result0);
		EXPECT_NEAR((expectedResult1 - expectedResult0).norm(), distance, epsilon);
		EXPECT_TRUE(expectedResult0.isApprox(result0));
		EXPECT_TRUE(expectedResult1.isApprox(result1));
	}

	{
		SCOPED_TRACE("Turn around the first line");
		distance = distanceLineLine(line0.b, line0.a, line1.a, line1.b, &result0, &result1);
		EXPECT_NEAR((expectedResult1 - expectedResult0).norm(), distance, epsilon);
		EXPECT_TRUE(expectedResult0.isApprox(result0));
		EXPECT_TRUE(expectedResult1.isApprox(result1));
	}

	{
		SCOPED_TRACE("And switch lines again");
		distance = distanceLineLine(line0.b, line0.a, line1.a, line1.b, &result0, &result1);
		EXPECT_NEAR((expectedResult1 - expectedResult0).norm(), distance, epsilon);
		EXPECT_TRUE(expectedResult0.isApprox(result0));
		EXPECT_TRUE(expectedResult1.isApprox(result1));
	}



}

TEST_F(GeometryTest, DistanceLineLine)
{
	SizeType distance;
	VectorType p0, p1;

	// Trivial the same line compared against itself
	distance = distanceLineLine(plainSegment.a, plainSegment.b, plainSegment.a, plainSegment.b, &p0, &p1);
	EXPECT_NEAR(0.0, distance, epsilon);

	// Parallel Line
	Segment parallel = Segment(plainSegment.a + plainNormal * 2, plainSegment.b + plainNormal * 2);
	distance = distanceLineLine(plainSegment.a, plainSegment.b, parallel.a, parallel.b, &p0, &p1);
	EXPECT_NEAR(2.0, distance, epsilon);

	// Not quite parallel, trying to get below epsilon
	parallel = Segment(plainSegment.a + plainNormal * 2 , plainSegment.b + plainNormal * 2 - plainNormal * 1.0e-10);
	distance = distanceLineLine(plainSegment.a, plainSegment.b, parallel.a, parallel.b, &p0, &p1);
	EXPECT_NEAR(2.0, distance, epsilon);

	{
		SCOPED_TRACE("Intersecting Lines");
		LineLineCheckData data(plainLine, intersectingLine, plainLine.b, plainLine.b);
		checkLineLineDistance(data);
	}

	{
		SCOPED_TRACE("Non-intersecting Lines");
		// Non Intersecting Line, don't know a better way to design this case besides reimplementing line/line distance
		Segment line0(VectorType(0, -5, 0), VectorType(0, 5, 0));
		Segment line1(VectorType(-5, 5, 5), VectorType(5, 5, 5));
		checkLineLineDistance(LineLineCheckData(line0, line1, VectorType(0, 5, 0), VectorType(0, 5, 5)));
	}


	// Degenerate Cases
	{
		SCOPED_TRACE("Both lines degenerate");
		Segment line0(VectorType(0, -5, 0), VectorType(0, -5, 0));
		Segment line1(VectorType(5, 5, 5), VectorType(5, 5, 5));
		checkLineLineDistance(LineLineCheckData(line0, line1, line0.a, line1.a));
	}

	{
		SCOPED_TRACE("Only one line degenerate");
		VectorType offLinePoint = plainSegment.a + plainSegment.ab * 0.5 + plainNormal * 1.5;
		LineLineCheckData data(plainSegment, Segment(offLinePoint, offLinePoint),
							   plainSegment.pointOnLine(0.5), offLinePoint);
		checkLineLineDistance(data);
	}

	{
		SCOPED_TRACE("Orthogonal Lines intersecting");
		Segment line0(VectorType(0, -5, 0), VectorType(0, 5, 0));
		VectorType v(3, 4, 5);
		VectorType n = line0.ab.cross(v);
		n.normalize();
		Segment line1(line0.a + n, line0.a - n);
		checkLineLineDistance(LineLineCheckData(line0, line1, line0.a, line0.a));
	}

	{
		SCOPED_TRACE("Orthogonal Lines non intersecting");
		Segment line0(VectorType(0, -5, 0), VectorType(0, 5, 0));
		VectorType v(3, 4, 5);
		VectorType n = line0.ab.cross(v);
		n.normalize();
		VectorType n2 = line0.ab.cross(n);
		n2.normalize();
		Segment line1(line0.a + n + n2, line0.a - n + n2);
		checkLineLineDistance(LineLineCheckData(line0, line1, line0.a, line0.a + n2));
	}

	{
		SCOPED_TRACE("not quite orthogonal Lines non intersecting ");
		Segment line0(VectorType(0, -5, 0), VectorType(0, 5, 0));
		VectorType v(3, 4, 5);
		VectorType n = line0.ab.cross(v);
		n.normalize();
		VectorType n2 = line0.ab.cross(n);
		n2.normalize();
		Segment line1(line0.a + n + n2 + line0.ab * 0.01, line0.a - n + n2 - line0.ab * 0.01);
		checkLineLineDistance(LineLineCheckData(line0, line1, line0.a, line0.a + n2));
	}

}



struct SegmentData
{
	Segment segment0;
	Segment segment1;
	VectorType p0;
	VectorType p1;
	SegmentData() {}
	SegmentData(Segment seg0, Segment seg1, VectorType vec0, VectorType vec1) :
		segment0(seg0), segment1(seg1), p0(vec0), p1(vec1) {}
};


void testSegmentDistance(const SegmentData& segmentData, const std::string& info, size_t i)
{
	SizeType distance;
	VectorType p0, p1;

	// The expected distance should be the distance between the two points that were
	// reported as being the closes ones
	SizeType expectedDistance = (segmentData.p1 - segmentData.p0).norm();

	distance = distanceSegmentSegment(segmentData.segment0.a, segmentData.segment0.b,
									  segmentData.segment1.a, segmentData.segment1.b, &p0, &p1);
	EXPECT_NEAR(expectedDistance, distance, 1e-8) << "for " << info << " at index " << i;
	EXPECT_TRUE(eigenEqual(segmentData.p0, p0)) << "for " << info << " at index " << i;
	EXPECT_TRUE(eigenEqual(segmentData.p1, p1)) << "for " << info << " at index " << i;

	distance = distanceSegmentSegment(segmentData.segment1.a, segmentData.segment1.b,
									  segmentData.segment0.a, segmentData.segment0.b, &p0, &p1);
	EXPECT_NEAR(expectedDistance, distance, 1e-8) << "for " << info << " at index " << i;
	EXPECT_TRUE(eigenEqual(segmentData.p1, p0)) << "for " << info << " at index " << i;
	EXPECT_TRUE(eigenEqual(segmentData.p0, p1)) << "for " << info << " at index " << i;
}



TEST_F(GeometryTest, DistanceSegmentSegment)
{
	SizeType distance;
	VectorType p0, p1;

	// Intersecting segments
	// Intersecting inside
	VectorType closestPoint = plainSegment.pointOnLine(0.5);
	Segment otherSegment(closestPoint + plainNormal, closestPoint - plainNormal);

	distance = distanceSegmentSegment(plainSegment.a, plainSegment.b, otherSegment.a, otherSegment.b, &p0, &p1);
	EXPECT_NEAR(0.0, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint, p0));
	EXPECT_TRUE(eigenEqual(closestPoint, p1));

	// Explode the cases
	std::vector<SegmentData> segments;

	// The following series test the segment to segment distance for
	// a) coplanar segments
	// b) non coplanar segments
	// for each of the groups multiple pairs of segments are testes in various configurations, where the
	// projections either intersect or don't, this should cover all the segments that are used in the
	// algorithm, inside testSegmentDistance the segments are also swapped around and tested against each other

	// <0> Intersecting outside past b with segment straddling the line
	closestPoint = plainSegment.pointOnLine(1.5);
	otherSegment = Segment(closestPoint + plainNormal, closestPoint - plainNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, closestPoint));

	// <1> segment not straddling, the correct points on the edges of the segments should get picked
	otherSegment = Segment(closestPoint + plainNormal, closestPoint + plainNormal * 2);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, otherSegment.a));

	// <2> segment not straddling, reverse the order of the points, reverse the side where the other segments falls
	otherSegment = Segment(closestPoint - plainNormal * 2, closestPoint - plainNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, otherSegment.b));

	// Go to the other side of the segment
	closestPoint = plainSegment.pointOnLine(-0.5);
	// <3> Straddling, there is actual an intersection
	otherSegment = Segment(closestPoint + plainNormal, closestPoint - plainNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, closestPoint));

	// <4> segment not straddling, the correct points on the edges of the segments should get picked
	otherSegment = Segment(closestPoint + plainNormal, closestPoint + plainNormal * 2);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, otherSegment.a));


	// <5> segment not straddling, reverse the order of the points, reverse the side where the other segments falls
	otherSegment = Segment(closestPoint - plainNormal * 2, closestPoint - plainNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, otherSegment.b));


	// Repeat the same sequence for the segments as they are not touching
	VectorType otherNormal = plainSegment.ab.cross(plainNormal);

	// <6> segment projections intersect
	closestPoint = plainSegment.pointOnLine(0.5) + plainNormal * 3;
	otherSegment = Segment(closestPoint + otherNormal, closestPoint - otherNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.pointOnLine(0.5), closestPoint));

	// <7> go past the end of the segment but straddle the line (T intersection)
	closestPoint = plainSegment.pointOnLine(1.5) + plainNormal * 3;
	otherSegment = Segment(closestPoint + otherNormal, closestPoint - otherNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, closestPoint));

	// <8> go past the end of the segment not straddling the line anymore
	otherSegment = Segment(closestPoint + otherNormal, closestPoint + otherNormal * 2);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, otherSegment.a));

	// <9> go past the end of the on the other side, switching up endpoints
	otherSegment = Segment(closestPoint - otherNormal * 2, closestPoint - otherNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, otherSegment.b));

	// Repeat for the other end of the base segment
	// <10> go past the end of the segment but straddle the line (T intersection)
	closestPoint = plainSegment.pointOnLine(-2.0) + plainNormal * 3;
	otherSegment = Segment(closestPoint + otherNormal, closestPoint - otherNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, closestPoint));

	// <11> go past the end of the segment not straddling the line anymore
	otherSegment = Segment(closestPoint + otherNormal, closestPoint + otherNormal * 2);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, otherSegment.a));

	// <12> go past the end of the on the other side, switching up endpoints
	otherSegment = Segment(closestPoint - otherNormal * 2, closestPoint - otherNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, otherSegment.b));

	// <13> projections intersect, short segments
	const VectorType aPoint = VectorType(1, 0, 0);
	const SizeType shortLength = 0.0001;
	const Segment shortSegment = Segment(VectorType(0, -shortLength/2, 0), VectorType(0, shortLength/2, 0));
	const VectorType shortSegmentNormal = shortSegment.ab.cross(aPoint).normalized();
	const Segment otherShortSegment = Segment(aPoint, aPoint + shortLength * shortSegmentNormal);
	segments.push_back(SegmentData(shortSegment, otherShortSegment,
		shortSegment.pointOnLine(0.5), otherShortSegment.a));

	for (size_t i = 0; i < segments.size(); ++i)
	{
		testSegmentDistance(segments[i], "basic cases", i);
	}

	// Parallel Segments
	otherSegment = Segment(plainSegment.a + plainNormal * 4, plainSegment.b + plainNormal * 4);
	distance = distanceSegmentSegment(plainSegment.a, plainSegment.b, otherSegment.a, otherSegment.b, &p0, &p1);
	EXPECT_NEAR(4.0, distance, epsilon);
	// What should the points be here ?

	segments.clear();

	// <0> parallel, non-overlapping
	closestPoint = plainSegment.a;
	const Vector3d segmentDirection = plainSegment.a - plainSegment.b;
	otherSegment = Segment(closestPoint + plainNormal * 4 + 2 * segmentDirection,
		closestPoint + plainNormal * 4 + 4 * segmentDirection);
	distance = distanceSegmentSegment(plainSegment.a, plainSegment.b, otherSegment.a, otherSegment.b, &p0, &p1);
	segments.push_back(SegmentData(plainSegment, otherSegment, closestPoint, otherSegment.a));

	// Anti-parallel Segments
	otherSegment = Segment(plainSegment.b + plainNormal * 4, plainSegment.a + plainNormal * 4);
	distance = distanceSegmentSegment(plainSegment.a, plainSegment.b, otherSegment.a, otherSegment.b, &p0, &p1);
	EXPECT_NEAR(4.0, distance, epsilon);

	// <1> anti-parallel, non-overlapping
	closestPoint = plainSegment.a;
	otherSegment = Segment(closestPoint + plainNormal * 4 + 4 * segmentDirection,
		closestPoint + plainNormal * 4 + 2 * segmentDirection);
	distance = distanceSegmentSegment(plainSegment.a, plainSegment.b, otherSegment.a, otherSegment.b, &p0, &p1);
	segments.push_back(SegmentData(plainSegment, otherSegment, closestPoint, otherSegment.b));

	for (size_t i = 0; i < segments.size(); ++i)
	{
		testSegmentDistance(segments[i], "parallel cases", i);
	}

	segments.clear();

	// The closest points are some assumptions, it looks like the algorithm is slanted towards
	// <0> the beginning points of the segments for this
	closestPoint = plainSegment.pointOnLine(0.5);
	otherSegment = Segment(closestPoint + plainNormal * 4, closestPoint + plainNormal * 8);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.pointOnLine(0.5), otherSegment.a));

	// <1> Move past the end of the segment on the far end
	closestPoint = plainSegment.pointOnLine(1.5);
	otherSegment = Segment(closestPoint + plainNormal * 4, closestPoint + plainNormal * 8);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, otherSegment.a));

	// <2> Move past the end of the segment on the near end
	closestPoint = plainSegment.pointOnLine(-2.0);
	otherSegment = Segment(closestPoint - plainNormal * 8, closestPoint - plainNormal * 4);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, otherSegment.b));


	// Degenerate cases delegate to PointSegDistance, just some spotChecks
	// <3> On the segment
	closestPoint = plainSegment.pointOnLine(0.5);
	otherSegment = Segment(closestPoint, closestPoint);
	segments.push_back(SegmentData(plainSegment, otherSegment, closestPoint, closestPoint));

	// <4> off the segment
	closestPoint = plainSegment.pointOnLine(1.5) + plainNormal * 4 + otherNormal * 10;
	otherSegment = Segment(closestPoint, closestPoint);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, closestPoint));


	for (size_t i = 0; i < segments.size(); ++i)
	{
		testSegmentDistance(segments[i], "other cases", i);
	}
}

TEST_F(GeometryTest, DistancePointTriangle)
{
	double distance;
	VectorType closestPoint;
	VectorType result;
	VectorType inputPoint;

	// Trivial, point on triangle
	inputPoint = VectorType(0, 0, 0);
	distance = distancePointTriangle(inputPoint, tri.v0, tri.v1, tri.v2, &result);
	EXPECT_NEAR(0.0, distance, epsilon);
	EXPECT_TRUE(eigenEqual(inputPoint, result));

	distance = distancePointTriangle(tri.v1, tri.v0, tri.v1, tri.v2, &result);
	EXPECT_NEAR(0.0, distance, epsilon);
	EXPECT_TRUE(eigenEqual(tri.v1, result));

	// Closest Point is inside Triangle
	closestPoint = tri.v0 + tri.v0v1 * 0.3 + tri.v0v2 * 0.7;
	inputPoint = closestPoint + tri.n * 2.5;
	distance = distancePointTriangle(inputPoint, tri.v0, tri.v1, tri.v2, &result);
	EXPECT_NEAR(2.5, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint, result));

	// other side
	inputPoint = closestPoint - tri.n * 3.5;
	distance = distancePointTriangle(inputPoint, tri.v0, tri.v1, tri.v2, &result);
	EXPECT_NEAR(3.5, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint, result));

	// Test the Point close to a triangle Edge cases
	// Point closest to edge v0v1
	double expectedDistance;
	inputPoint = tri.v0 +  tri.v0v1 * 0.5 - tri.v0v2 + tri.n;
	distance = distancePointTriangle(inputPoint, tri.v0, tri.v1, tri.v2, &result);
	expectedDistance = distancePointSegment(inputPoint, tri.v0, tri.v1, &closestPoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint, result));

	// Point closest to edge v0v2
	inputPoint = tri.v0 + tri.v0v2 * 0.3 - tri.v0v1 + tri.n * 2;
	distance = distancePointTriangle(inputPoint, tri.v0, tri.v1, tri.v2, &result);
	expectedDistance = distancePointSegment(inputPoint, tri.v0, tri.v2, &closestPoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint, result));

	// Point closest to edge v1v2
	inputPoint = tri.v1 + (tri.v2 - tri.v1) * .75 + tri.v0v1 * 0.2 + tri.n;
	distance = distancePointTriangle(inputPoint, tri.v0, tri.v1, tri.v2, &result);
	expectedDistance = distancePointSegment(inputPoint, tri.v1, tri.v2, &closestPoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint, result));

	// Point closest to point v0
	inputPoint = tri.v0 - tri.v0v1 - tri.v0v2 * 0.5 - tri.n;
	distance = distancePointTriangle(inputPoint, tri.v0, tri.v1, tri.v2, &result);
	expectedDistance = (tri.v0 - inputPoint).norm();
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(tri.v0, result));

	// Point closest to point v1
	inputPoint = tri.v1 + tri.v0v1 + (tri.v1 - tri.v2) * 2.0 - tri.n * 2.0;
	distance = distancePointTriangle(inputPoint, tri.v0, tri.v1, tri.v2, &result);
	expectedDistance = (tri.v1 - inputPoint).norm();
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(tri.v1, result));

	// Point closest to point v2
	inputPoint = tri.v2 + tri.v0v2 + (tri.v2 - tri.v1) * 3.0 - tri.n * 1.5;
	distance = distancePointTriangle(inputPoint, tri.v0, tri.v1, tri.v2, &result);
	expectedDistance = (tri.v2 - inputPoint).norm();
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(tri.v2, result));

	// Degenerate Edges
	// Edge v0v1
	distance = distancePointTriangle(inputPoint,
			   tri.v0, (tri.v0 + tri.v0v1 * epsilon * 0.01).eval(), tri.v2,
			   &result);
	expectedDistance = distancePointSegment(inputPoint, tri.v0, tri.v2, &closestPoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint, result));

	// Edge v0v2
	distance = distancePointTriangle(inputPoint,
			   (tri.v2 - tri.v0v2 * epsilon * 0.01).eval(), tri.v1 , tri.v2,
			   &result);
	expectedDistance = distancePointSegment(inputPoint, tri.v1, tri.v2, &closestPoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint, result));

	// Edge v1v2
	distance = distancePointTriangle(inputPoint, tri.v0, tri.v1 , tri.v1, &result);
	expectedDistance = distancePointSegment(inputPoint, tri.v1, tri.v0, &closestPoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint, result));
}

TEST_F(GeometryTest, PointInsideTriangleWithNormal)
{
	EXPECT_TRUE(isPointInsideTriangle(tri.v0, tri.v0, tri.v1, tri.v2, tri.n));
	EXPECT_TRUE(isPointInsideTriangle(tri.v1, tri.v0, tri.v1, tri.v2, tri.n));
	EXPECT_TRUE(isPointInsideTriangle(tri.v2, tri.v0, tri.v1, tri.v2, tri.n));

	VectorType inputPoint = tri.v0 + tri.v0v1 * 0.2;
	EXPECT_TRUE(isPointInsideTriangle(inputPoint, tri.v0, tri.v1, tri.v2, tri.n));
	inputPoint += tri.v0v2 * 0.5;
	EXPECT_TRUE(isPointInsideTriangle(inputPoint, tri.v0, tri.v1, tri.v2, tri.n));

	inputPoint = tri.v0 + tri.v0v1 * 1.5;
	EXPECT_FALSE(isPointInsideTriangle(inputPoint, tri.v0, tri.v1, tri.v2, tri.n));
	EXPECT_FALSE(isPointInsideTriangle(inputPoint, tri.v1, tri.v1, tri.v2, tri.n));

	inputPoint = tri.v0 + tri.v0v2 * 2 + tri.v0v1 * 2;
	EXPECT_FALSE(isPointInsideTriangle(inputPoint, tri.v0, tri.v1, tri.v2, tri.n));

}

TEST_F(GeometryTest, PointInsideTriangleWithoutNormal)
{
	EXPECT_TRUE(isPointInsideTriangle(tri.v0, tri.v0, tri.v1, tri.v2));
	EXPECT_TRUE(isPointInsideTriangle(tri.v1, tri.v0, tri.v1, tri.v2));
	EXPECT_TRUE(isPointInsideTriangle(tri.v2, tri.v0, tri.v1, tri.v2));

	VectorType inputPoint = tri.v0 + tri.v0v1 * 0.2;
	EXPECT_TRUE(isPointInsideTriangle(inputPoint, tri.v0, tri.v1, tri.v2));
	inputPoint += tri.v0v2 * 0.5;
	EXPECT_TRUE(isPointInsideTriangle(inputPoint, tri.v0, tri.v1, tri.v2));

	inputPoint = tri.v0 + tri.v0v1 * 1.5;
	EXPECT_FALSE(isPointInsideTriangle(inputPoint, tri.v0, tri.v1, tri.v2));
	EXPECT_FALSE(isPointInsideTriangle(inputPoint, tri.v1, tri.v1, tri.v2));

	inputPoint = tri.v0 + tri.v0v2 * 2 + tri.v0v1 * 2;
	EXPECT_FALSE(isPointInsideTriangle(inputPoint, tri.v0, tri.v1, tri.v2));
}

TEST_F(GeometryTest, Coplanarity)
{
	struct CoplanarityTestCandidate
	{
		bool expected;
		std::array<Vector3d, 4> points;
	};

	CoplanarityTestCandidate candidates[] =
	{
		{true, Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)},
		{true, Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), Vector3d(3.7, 0.0, 0.0)},
		{true, Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), Vector3d(2.3, 0.0, 0.0), Vector3d(3.7, 0.0, 0.0)},
		{true, Vector3d(0.0, 0.0, 0.0), Vector3d(1.1, 0.0, 0.0), Vector3d(2.3, 0.0, 0.0), Vector3d(3.7, 0.0, 0.0)},
		{true, Vector3d(0.0, 0.0, 0.0), Vector3d(1.1, 1.5, 0.0), Vector3d(2.3, 0.0, 0.0), Vector3d(3.7, 0.0, 0.0)},
		{true, Vector3d(0.0, 0.0, 0.0), Vector3d(1.1, 1.5, 0.0), Vector3d(2.3, 0.0, 0.0), Vector3d(3.7, 3.0, 0.0)},
		{false, Vector3d(0.0, 0.0, 1.0), Vector3d(1.1, 1.5, 0.0), Vector3d(2.3, 0.0, 0.0), Vector3d(3.7, 3.0, 0.0)},
		{false, Vector3d(0.0, 0.0, 0.0), Vector3d(1.1, 1.5, 1.1), Vector3d(2.3, 0.0, 0.0), Vector3d(3.7, 3.0, 0.0)},
		{false, Vector3d(0.0, 0.0, 0.0), Vector3d(1.1, 1.5, 0.0), Vector3d(2.3, 0.0, 7.7), Vector3d(3.7, 3.0, 0.0)},
		{false, Vector3d(0.0, 0.0, 0.0), Vector3d(1.1, 1.5, 0.0), Vector3d(2.3, 0.0, 0.0), Vector3d(3.7, 3.0, -9.6)},

		{true, Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0), Vector3d(12.3, -41.3, 0.0)},
		{false, Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0), Vector3d(12.3, -41.3, 4.0)},

		{false, Vector3d(10932.645, 43.1987, -0.009874245),
				Vector3d(53432.4, -9.87243, 654.31),
				Vector3d(28.71, 0.005483927, 2.34515),
				Vector3d(5897.1, -5.432, 512152.7654)}
	};

	for (auto candidate = std::begin(candidates); candidate != std::end(candidates); ++candidate)
	{
		EXPECT_EQ(candidate->expected, isCoplanar(candidate->points[0],
												  candidate->points[1],
												  candidate->points[2],
												  candidate->points[3]))
			<< "Candidate points were [" << candidate->points[0].transpose() << "], ["
										 << candidate->points[1].transpose() << "], ["
										 << candidate->points[2].transpose() << "], ["
										 << candidate->points[3].transpose() << "]";
	}
}

typedef std::tuple<Segment, MockTriangle, VectorType, bool> SegTriIntersectionData;
::testing::AssertionResult checkSegTriIntersection(const SegTriIntersectionData& data)
{
	std::stringstream errorMessage;
	Segment segment = std::get<0>(data);
	MockTriangle tri = std::get<1>(data);
	VectorType expectedClosestPoint = std::get<2>(data);
	bool expectedResult = std::get<3>(data);
	VectorType closestPoint;

	bool result = doesCollideSegmentTriangle(segment.a, segment.b, tri.v0, tri.v1, tri.v2, tri.n, &closestPoint);
	if (result != expectedResult)
	{
		errorMessage << "Intersection result does not match should be: " << expectedResult << " but got " <<
					 result << std::endl;
	};
	if (expectedResult)
	{
		if (! expectedClosestPoint.isApprox(closestPoint))
		{
			errorMessage << "Closest Point was expected to be " << expectedClosestPoint << " but is " <<
						 closestPoint << std::endl;
		}
	}
	else
	{
		errorMessage << eigenAllNan(closestPoint).message();
	}

	if (errorMessage.str() == "")
	{
		return ::testing::AssertionSuccess();
	}
	else
	{
		return ::testing::AssertionFailure() << errorMessage.str();
	}
}

TEST_F(GeometryTest, SegmentTriangleIntersection)
{
	VectorType closestPoint;
	VectorType intersectionPoint = tri.pointInTriangle(0.2, 0.7);
	Segment intersecting(intersectionPoint - tri.n * 2, intersectionPoint + tri.n * 2);

	SegTriIntersectionData data;

	data = SegTriIntersectionData(intersecting, tri, intersectionPoint, true);
	EXPECT_TRUE(checkSegTriIntersection(data));

	intersecting.a = intersectionPoint + tri.n * 4;
	data = SegTriIntersectionData(intersecting, tri, intersectionPoint, false);
	EXPECT_TRUE(checkSegTriIntersection(data));

	// in the plane of the triangle
	intersecting = Segment(intersectionPoint, intersectionPoint + tri.v0v1 + tri.v1v2);
	data = SegTriIntersectionData(intersecting, tri, intersectionPoint, true);
	EXPECT_TRUE(checkSegTriIntersection(data));

	intersecting = Segment(intersectionPoint + tri.v0v1 + tri.v1v2, intersectionPoint);
	data = SegTriIntersectionData(intersecting, tri, intersectionPoint, true);
	EXPECT_TRUE(checkSegTriIntersection(data));

	intersecting = Segment(intersectionPoint + tri.v0v1 + tri.v1v2, intersectionPoint + 2 * tri.v0v1 + tri.v1v2);
	data = SegTriIntersectionData(intersecting, tri, intersectionPoint, false);
	EXPECT_TRUE(checkSegTriIntersection(data));

	// Slanting but intersecting
	// Point On triangle
	intersecting = Segment(intersectionPoint, intersectionPoint + tri.n * 2 + tri.v0v1 * 2);
	data = SegTriIntersectionData(intersecting, tri, intersectionPoint, true);
	EXPECT_TRUE(checkSegTriIntersection(data));

	// Intersection in Triangle
	intersecting = Segment(intersectionPoint - tri.n * 2 - tri.v1v2 * 2, intersectionPoint + 2 * tri.n + tri.v1v2 * 2);
	data = SegTriIntersectionData(intersecting, tri, intersectionPoint, true);
	EXPECT_TRUE(checkSegTriIntersection(data));

	// Intersection not on Segment
	intersecting = Segment(intersectionPoint + tri.n * 4 + tri.v1v2 * 4, intersectionPoint + 2 * tri.n + tri.v1v2 * 2);
	data = SegTriIntersectionData(intersecting, tri, intersectionPoint, false);
	EXPECT_TRUE(checkSegTriIntersection(data));

	// Normal segment through one edge
	VectorType pointOnEdge = tri.v0 + tri.v0v1 * 0.5;
	intersecting = Segment(pointOnEdge + tri.n, pointOnEdge - tri.n);
	data = SegTriIntersectionData(intersecting, tri, pointOnEdge, true);
	EXPECT_TRUE(checkSegTriIntersection(data));

	intersecting = Segment(pointOnEdge + tri.n, pointOnEdge);
	data = SegTriIntersectionData(intersecting, tri, pointOnEdge, true);
	EXPECT_TRUE(checkSegTriIntersection(data));

	intersecting = Segment(pointOnEdge + tri.n * 3, pointOnEdge + tri.n * 4);
	data = SegTriIntersectionData(intersecting, tri, pointOnEdge, false);
	EXPECT_TRUE(checkSegTriIntersection(data));

	intersecting = Segment(pointOnEdge + tri.n + tri.v0v1 * 0.5, pointOnEdge);
	data = SegTriIntersectionData(intersecting, tri, pointOnEdge, true);
	EXPECT_TRUE(checkSegTriIntersection(data));

	// Segment away from the triangle
	intersecting = Segment(tri.v0 - tri.v0v1 - tri.v1v2, tri.v0 - tri.n * 3.0 - tri.v0v1 - tri.v1v2);
	data = SegTriIntersectionData(intersecting, tri, pointOnEdge, false);
	EXPECT_TRUE(checkSegTriIntersection(data));
}

TEST_F(GeometryTest, distancePointPlane)
{
	MockTriangle triangle(VectorType(3, 4, 5), VectorType(5, 5, 5), VectorType(10, 5, 2));
	VectorType pointInTriangle = triangle.v0 + triangle.v0v1 * 0.4;
	double d = -triangle.n.dot(triangle.v0);
	VectorType point = pointInTriangle;
	VectorType projectionPoint;
	double distance = distancePointPlane(point, triangle.n, d, &projectionPoint);
	EXPECT_NEAR(0.0, distance, epsilon);
	EXPECT_TRUE(pointInTriangle.isApprox(projectionPoint));

	point = pointInTriangle + triangle.n * 2;
	distance = distancePointPlane(point, triangle.n, d, &projectionPoint);
	EXPECT_NEAR(2.0, distance, epsilon);
	EXPECT_TRUE(pointInTriangle.isApprox(projectionPoint));

	point = pointInTriangle - triangle.n * 3;
	distance = distancePointPlane(point, triangle.n, d, &projectionPoint);
	EXPECT_NEAR(-3.0, distance, epsilon);
	EXPECT_TRUE(pointInTriangle.isApprox(projectionPoint));
}

typedef std::tuple<Segment, VectorType, double, VectorType, VectorType, int> SegmentPlaneData;
void checkSegmentPlanDistance(const SegmentPlaneData& data)
{
	Segment seg = std::get<0>(data);
	VectorType n = std::get<1>(data);
	double d = std::get<2>(data);
	VectorType expectedSegmentPoint = std::get<3>(data);
	VectorType expectedPlanePoint = std::get<4>(data);
	// The sign of the expected distance [1|-1|0], you must use 0 for expected 0
	int sign = std::get<5>(data);
	double distance;
	VectorType segResultPoint, planeResultPoint;
	distance = distanceSegmentPlane(seg. a, seg.b, n, d, &segResultPoint, &planeResultPoint);
	EXPECT_NEAR((planeResultPoint - segResultPoint).norm(), std::abs(distance), epsilon);
	EXPECT_TRUE(distance * sign > 0 || distance == static_cast<double>(sign));
	EXPECT_TRUE(expectedSegmentPoint.isApprox(segResultPoint));
	EXPECT_TRUE(expectedPlanePoint.isApprox(planeResultPoint));
}

TEST_F(GeometryTest, SegmentPlaneDistance)
{
	MockTriangle triangle(VectorType(3, 4, 5), VectorType(5, 5, 5), VectorType(10, 5, 2));
	double d = -triangle.n.dot(triangle.v0);
	VectorType intersectionPoint = triangle.pointInTriangle(0.2, 0.7);
	Segment seg(intersectionPoint - triangle.n * 2, intersectionPoint + triangle.n * 2);

	VectorType segResultPoint, planeResultPoint;

	{
		SCOPED_TRACE("Segment intersects Plane");
		checkSegmentPlanDistance(SegmentPlaneData(seg, triangle.n, d, intersectionPoint, intersectionPoint, 0));
	}

	{
		SCOPED_TRACE("Segment above plane, segment intersection should be point a");
		seg = Segment(intersectionPoint + triangle.n * 2, intersectionPoint + triangle.n * 3);
		distanceSegmentPlane(seg.a, seg.b, triangle.n, d, &segResultPoint, &planeResultPoint);
		checkSegmentPlanDistance(SegmentPlaneData(seg, triangle.n, d, seg.a, intersectionPoint, 1));
	}

	{
		SCOPED_TRACE("Segment below plane, segment intersection should be point a");
		seg = Segment(intersectionPoint - triangle.n * 3, intersectionPoint - triangle.n * 2);
		distanceSegmentPlane(seg.a, seg.b, triangle.n, d, &segResultPoint, &planeResultPoint);
		checkSegmentPlanDistance(SegmentPlaneData(seg, triangle.n, d, seg.b, intersectionPoint, -1));
	}

	{
		SCOPED_TRACE("Segment below plane, segment intersection should be point a, reverse case from above");
		seg = Segment(intersectionPoint - triangle.n * 2, intersectionPoint - triangle.n * 3);
		checkSegmentPlanDistance(SegmentPlaneData(seg, triangle.n, d, seg.a, intersectionPoint, -1));
	}

	{
		SCOPED_TRACE("Segment coplanar with plane");
		seg = Segment(intersectionPoint - triangle.v0v1, intersectionPoint + triangle.v0v1);
		checkSegmentPlanDistance(SegmentPlaneData(seg, triangle.n, d, intersectionPoint, intersectionPoint, 0));
	}

	{
		SCOPED_TRACE("Segment parallel with plane");
		seg = Segment(intersectionPoint - triangle.v0v1 + triangle.n * 2.0,
					  intersectionPoint + triangle.v0v1 + triangle.n * 2.0);
		checkSegmentPlanDistance(SegmentPlaneData(seg, triangle.n, d, seg.pointOnLine(0.5), intersectionPoint, 1));
	}

	{
		SCOPED_TRACE("Segment parallel with plane but on the other side");
		seg = Segment(intersectionPoint - triangle.v0v1 - triangle.n * 2.0,
					  intersectionPoint + triangle.v0v1 - triangle.n * 2.0);
		checkSegmentPlanDistance(SegmentPlaneData(seg, triangle.n, d, seg.pointOnLine(0.5), intersectionPoint, -1));
	}
}

typedef std::tuple<MockTriangle, VectorType, double, VectorType, VectorType, int> TriPlaneData;
void checkTriPlaneDistance(const TriPlaneData& data)
{
	MockTriangle tri = std::get<0>(data);
	VectorType n = std::get<1>(data);
	double d = std::get<2>(data);
	VectorType expectedTrianglePoint = std::get<3>(data);
	VectorType expectedPlanePoint = std::get<4>(data);
	int sign = std::get<5>(data);
	VectorType triangleResultPoint, planeResultPoint;
	double distance;

	distance = distanceTrianglePlane(tri.v0, tri.v1, tri.v2, n, d, &triangleResultPoint, &planeResultPoint);
	EXPECT_NEAR((planeResultPoint - triangleResultPoint).norm(), std::abs(distance), epsilon);
	EXPECT_TRUE((sign == 0 && std::abs(distance) <= epsilon) || (distance * sign > 0));
	EXPECT_TRUE(expectedTrianglePoint.isApprox(triangleResultPoint));
	EXPECT_TRUE(expectedPlanePoint.isApprox(planeResultPoint));
}

TEST_F(GeometryTest, TrianglePlaneTest)
{
	MockTriangle triangle(VectorType(3, 4, 5), VectorType(5, 5, 5), VectorType(10, 5, 2));
	// Start with the coplanar case
	double d = -triangle.n.dot(triangle.v0);
	double distance;
	VectorType intersectionPoint0;
	VectorType intersectionPoint1;

	// Coplanar
	VectorType third = (triangle.v0 + triangle.v1 + triangle.v2) / 3.0;
	distance = distanceTrianglePlane(triangle.v0, triangle.v1, triangle.v2, triangle.n, d,
									 &intersectionPoint0, &intersectionPoint1);
	EXPECT_NEAR(0.0, distance, epsilon);
	EXPECT_TRUE(third.isApprox(intersectionPoint0));
	EXPECT_TRUE(third.isApprox(intersectionPoint1));

	VectorType pointOnPlane = (triangle.v0 + triangle.v1 + triangle.v2) / 3.0;

	{
		SCOPED_TRACE("Coplanar Case");
		MockTriangle target(triangle.v0 , triangle.v1 , triangle.v2);
		checkTriPlaneDistance(TriPlaneData(target, triangle.n, d, pointOnPlane, pointOnPlane, 0));
	}

	{
		SCOPED_TRACE("Parallel, below the plane");
		MockTriangle target(triangle.v0 - triangle.n * 3, triangle.v1 - triangle.n * 3, triangle.v2 - triangle.n * 3);
		checkTriPlaneDistance(TriPlaneData(target, triangle.n, d, pointOnPlane - triangle.n * 3, pointOnPlane, -1));
	}

	{
		SCOPED_TRACE("Parallel, above the plane");
		MockTriangle target(triangle.v0 + triangle.n, triangle.v1 + triangle.n, triangle.v2 + triangle.n);
		checkTriPlaneDistance(TriPlaneData(target, triangle.n, d, pointOnPlane + triangle.n, pointOnPlane, 1));
	}

	{
		SCOPED_TRACE("Not Intersecting, triangle.v0 is closest above the plane");
		MockTriangle target(triangle.v0 + triangle.n * 2, triangle.v1 + triangle.n * 3, triangle.v2 + triangle.n * 3);
		checkTriPlaneDistance(TriPlaneData(target, triangle.n, d, target.v0, triangle.v0, 1));
	}

	{
		SCOPED_TRACE("Not Intersecting, triangle.v1 is closest above the plane");
		MockTriangle target(triangle.v0 + triangle.n * 3, triangle.v1 + triangle.n * 2, triangle.v2 + triangle.n * 3);
		checkTriPlaneDistance(TriPlaneData(target, triangle.n, d, target.v1, triangle.v1, 1));
	}

	{
		SCOPED_TRACE("Not Intersecting, triangle.v2 is closest above the plane");
		MockTriangle target(triangle.v0 + triangle.n * 4, triangle.v1 + triangle.n * 3, triangle.v2 + triangle.n * 2);
		checkTriPlaneDistance(TriPlaneData(target, triangle.n, d, target.v2, triangle.v2, 1));
	}

	{
		SCOPED_TRACE("Not Intersecting, triangle.v0 is closest below the plane");
		MockTriangle target(triangle.v0 - triangle.n * 2, triangle.v1 - triangle.n * 3, triangle.v2 - triangle.n * 3);
		checkTriPlaneDistance(TriPlaneData(target, triangle.n, d, target.v0, triangle.v0, -1));
	}

	{
		SCOPED_TRACE("Not Intersecting, triangle.v1 is closest below the plane");
		MockTriangle target(triangle.v0 - triangle.n * 4, triangle.v1 - triangle.n * 2, triangle.v2 - triangle.n * 3);
		checkTriPlaneDistance(TriPlaneData(target, triangle.n, d, target.v1, triangle.v1, -1));
	}

	{
		SCOPED_TRACE("Not Intersecting, triangle.v2 is closest below the plane");
		MockTriangle target(triangle.v0 - triangle.n * 4, triangle.v1 - triangle.n * 3, triangle.v2 - triangle.n * 2);
		checkTriPlaneDistance(TriPlaneData(target, triangle.n, d, target.v2, triangle.v2, -1));
	}

	{
		SCOPED_TRACE("Triangle point on the plane");
		// Need to change the order of points for this to work ... strange ...
		MockTriangle target(triangle.v0 + triangle.n * 3, triangle.v2 + triangle.n * 3, triangle.v1);
		checkTriPlaneDistance(TriPlaneData(target, triangle.n, d, target.v2, target.v2, 0));
	}

	{
		SCOPED_TRACE("Triangle plane intersection with v0 being under the plane");
		MockTriangle target(triangle.v0 - triangle.n * 2, triangle.v1 + triangle.n * 2, triangle.v2 + triangle.n * 2);
		distance = distanceTrianglePlane(target.v0, target.v1, target.v2, triangle.n, d,
										 &intersectionPoint0, &intersectionPoint1);
		EXPECT_NEAR(0.0, distance, epsilon);
		EXPECT_TRUE(intersectionPoint0.isApprox(intersectionPoint1));
		EXPECT_TRUE(isPointInsideTriangle(intersectionPoint0, target.v0, target.v1, target.v2, target.n));
		EXPECT_TRUE(isPointInsideTriangle(intersectionPoint0, triangle.v0, triangle.v1, triangle.v2, triangle.n));
	}

	{
		SCOPED_TRACE("Triangle plane intersection with v0 and v1 being under the plane");
		MockTriangle target(triangle.v0 - triangle.n * 2, triangle.v1 - triangle.n * 2, triangle.v2 + triangle.n * 2);
		distance = distanceTrianglePlane(target.v0, target.v1, target.v2, triangle.n, d,
										 &intersectionPoint0, &intersectionPoint1);
		EXPECT_NEAR(0.0, distance, epsilon);
		EXPECT_TRUE(intersectionPoint0.isApprox(intersectionPoint1));
		EXPECT_TRUE(isPointInsideTriangle(intersectionPoint0, target.v0, target.v1, target.v2, target.n));
		EXPECT_TRUE(isPointInsideTriangle(intersectionPoint0, triangle.v0, triangle.v1, triangle.v2, triangle.n));
	}

	{
		SCOPED_TRACE("Triangle plane intersection with v2 being under the plane");
		MockTriangle target(triangle.v0 + triangle.n * 2, triangle.v1 + triangle.n * 2, triangle.v2 - triangle.n * 2);
		distance = distanceTrianglePlane(target.v0, target.v1, target.v2, triangle.n, d,
										 &intersectionPoint0, &intersectionPoint1);
		EXPECT_NEAR(0.0, distance, epsilon);
		EXPECT_TRUE(intersectionPoint0.isApprox(intersectionPoint1));
		EXPECT_TRUE(isPointInsideTriangle(intersectionPoint0, target.v0, target.v1, target.v2, target.n));
		EXPECT_TRUE(isPointInsideTriangle(intersectionPoint0, triangle.v0, triangle.v1, triangle.v2, triangle.n));
	}
}

TEST_F(GeometryTest, PlanePlaneDistance)
{
	// Simple test against same
	double d1 = -tri.n.dot(tri.v0);
	VectorType point0, point1;

	bool result = doesIntersectPlanePlane(tri.n, d1, tri.n, d1, &point0, &point1);
	EXPECT_FALSE(result);
	EXPECT_TRUE(eigenAllNan(point0));
	EXPECT_TRUE(eigenAllNan(point1));
	result = doesIntersectPlanePlane(tri.n, -2.0, tri.n, 8.8, &point0, &point1);
	EXPECT_FALSE(result);
	EXPECT_TRUE(eigenAllNan(point0));
	EXPECT_TRUE(eigenAllNan(point1));

	VectorType n2 = VectorType(5, 6, 7);
	n2.normalize();
	double d2 = -2;
	result = doesIntersectPlanePlane(tri.n, d1, n2, d2, &point0, &point1);
	VectorType output;
	EXPECT_TRUE(result);
	EXPECT_FALSE(eigenAllNan(point0));
	EXPECT_NEAR(0, distancePointPlane(point0, tri.n, d1, &output), epsilon);
	EXPECT_NEAR(0, distancePointPlane(point1, tri.n, d1, &output), epsilon);
	EXPECT_FALSE(eigenAllNan(point1));
	EXPECT_NEAR(0, distancePointPlane(point0, n2, d2, &output), epsilon);
	EXPECT_NEAR(0, distancePointPlane(point1, n2, d2, &output), epsilon);
}

typedef std::tuple<Segment, MockTriangle, VectorType, VectorType> SegTriDistanceData;
void checkSegTriDistance(const SegTriDistanceData& data)
{
	std::stringstream errorMessage;
	Segment segment = std::get<0>(data);
	MockTriangle tri = std::get<1>(data);
	VectorType expectedSegmentPoint = std::get<2>(data);
	VectorType expectedTrianglePoint = std::get<3>(data);
	double expectedDistance = (expectedSegmentPoint - expectedTrianglePoint).norm();
	double distance;
	VectorType segmentPoint, trianglePoint;

	distance = distanceSegmentTriangle(segment.a, segment.b, tri.v0, tri.v1, tri.v2, tri.n,
									   &segmentPoint, &trianglePoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(expectedSegmentPoint.isApprox(segmentPoint));
	EXPECT_TRUE(expectedTrianglePoint.isApprox(trianglePoint));


	// Repeat above with segment reversed
	distance = distanceSegmentTriangle(segment.b, segment.a, tri.v0, tri.v1, tri.v2, tri.n,
									   &segmentPoint, &trianglePoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(expectedSegmentPoint.isApprox(segmentPoint));
	EXPECT_TRUE(expectedTrianglePoint.isApprox(trianglePoint));
}
TEST_F(GeometryTest, SegmentTriangleDistance)
{
	Segment segment;
	VectorType intersection;
	{
		SCOPED_TRACE("Segment endpoint equivalent to triangle point");
		segment = Segment(tri.v0, tri.v1 + tri.n * 3);
		checkSegTriDistance(SegTriDistanceData(segment, tri, segment.a, tri.v0));
	}
	{
		SCOPED_TRACE("Segment endpoint inside triangle on triangle plane");
		segment = Segment(tri.pointInTriangle(0.5, 0.2), tri.pointInTriangle(2, 2) + tri.n * 4);
		checkSegTriDistance(SegTriDistanceData(segment, tri, segment.a, segment.a));
	}
	{
		SCOPED_TRACE("Intersection inside triangle");
		intersection = tri.pointInTriangle(0.5, 0.2);
		segment = Segment(intersection - tri.n * 4 - tri.v0v1, intersection + tri.n * 4 + tri.v0v1);
		checkSegTriDistance(SegTriDistanceData(segment, tri, intersection, intersection));
	}
	{
		SCOPED_TRACE("Segment endpoint on triangle edge");
		segment = Segment(tri.pointInTriangle(0.0, 0.2), tri.pointInTriangle(2, 2) + tri.n * 4);
		checkSegTriDistance(SegTriDistanceData(segment, tri, segment.a, segment.a));
	}
	{
		SCOPED_TRACE("intersection on triangle edge");
		intersection = tri.pointInTriangle(0, 0.2);
		segment = Segment(intersection - tri.n * 3 - tri.v1v2 * .5, intersection + tri.n * 3 + tri.v1v2 * .5);
		checkSegTriDistance(SegTriDistanceData(segment, tri, intersection, intersection));
	}
	{
		SCOPED_TRACE("segment endpoint is close to point inside of triangle");
		intersection = tri.pointInTriangle(0.5, 0.2);
		Segment seg(intersection, intersection + tri.n * 2 + tri.v0v1 * 3);
		segment = Segment(seg.a + tri.n * 0.1, seg.b + tri.n * 0.1);
		checkSegTriDistance(SegTriDistanceData(segment, tri, segment.a, intersection));
	}
	{
		SCOPED_TRACE("segment endpoint is close to triangle point v0");
		Segment seg(tri.v0, tri.v0 - tri.n * 2 - tri.v0v1 * 2);
		segment = Segment(seg.a - tri.n * 0.1, seg.b - tri.n * 0.1);
		checkSegTriDistance(SegTriDistanceData(segment, tri, segment.a, tri.v0));
	}
	{
		SCOPED_TRACE("segment endpoint is close to triangle point v1");
		Segment seg(tri.v1, tri.v1 - tri.n * 2 - tri.v0v1 * 2);
		segment = Segment(seg.a - tri.n * 0.1, seg.b - tri.n * 0.1);
		checkSegTriDistance(SegTriDistanceData(segment, tri, segment.a, tri.v1));
	}
	{
		SCOPED_TRACE("segment endpoint is close to triangle point v2");
		Segment seg(tri.v2, tri.v2 - tri.n * 2 - tri.v1v2 * 2);
		segment = Segment(seg.a - tri.n * 0.1, seg.b - tri.n * 0.1);
		checkSegTriDistance(SegTriDistanceData(segment, tri, segment.a, tri.v2));
	}
	{
		SCOPED_TRACE("segment endpoint is close to edge v0v1");
		intersection = tri.v0 + tri.v0v1 * 0.2;
		Segment seg(intersection, intersection + tri.n * 2);
		segment = Segment(seg.a + seg.ab * 0.01, seg.b + seg.ab * 0.01);
		checkSegTriDistance(SegTriDistanceData(segment, tri, segment.a, intersection));
	}
	{
		SCOPED_TRACE("segment endpoint is close to edge v0v2");
		intersection = tri.v0 + tri.v0v2 * 0.4;
		Segment seg(intersection, intersection + tri.n * 2);
		segment = Segment(seg.a + seg.ab * 0.01, seg.b + seg.ab * 0.01);
		checkSegTriDistance(SegTriDistanceData(segment, tri, segment.a , intersection));
	}
	{
		SCOPED_TRACE("segment endpoint is close to edge v1v2");
		intersection = tri.v1 + tri.v1v2 * 0.2;
		Segment seg(intersection, intersection + tri.n * 2);
		segment = Segment(seg.a + seg.ab * 0.01, seg.b + seg.ab * 0.01);
		checkSegTriDistance(SegTriDistanceData(segment, tri, segment.a, intersection));
	}
	{
		SCOPED_TRACE("point on segment is close to triangle vertex v0");
		segment = Segment(tri.v0 - tri.n * 3, tri.v0 + tri.n * 3);
		checkSegTriDistance(SegTriDistanceData(segment, tri, tri.v0, tri.v0));
	}
	{
		SCOPED_TRACE("point on segment is close to triangle vertex v1");
		segment = Segment(tri.v1 - tri.n * 3, tri.v1 + tri.n * 3);
		checkSegTriDistance(SegTriDistanceData(segment, tri, tri.v1, tri.v1));
	}
	{
		SCOPED_TRACE("point on segment is close to triangle vertex v2");
		segment = Segment(tri.v2 - tri.n * 3, tri.v2 + tri.n * 3);
		checkSegTriDistance(SegTriDistanceData(segment, tri, tri.v2, tri.v2));
	}
	{
		SCOPED_TRACE("point on segment is close to edge v0v1");
		intersection = tri.v0 + tri.v0v1 * 0.2;
		Segment seg(intersection - tri.n * 3, intersection + tri.n * 2);
		VectorType cross = tri.n.cross(tri.v0v1);
		segment = Segment(seg.a - cross * 0.01, seg.b - cross * 0.01);
		checkSegTriDistance(SegTriDistanceData(segment, tri, intersection - cross * 0.01, intersection));
	}
	{
		SCOPED_TRACE("point on segment is close to edge v0v2");
		intersection = tri.v0 + tri.v0v2 * 0.2;
		Segment seg(intersection - tri.n * 3, intersection + tri.n * 2);
		VectorType cross = tri.n.cross(tri.v0v2);
		segment = Segment(seg.a + cross * 0.01, seg.b + cross * 0.01);
		checkSegTriDistance(SegTriDistanceData(segment, tri, intersection + cross * 0.01, intersection));
	}
	{
		SCOPED_TRACE("point on segment is close to edge v1v2");
		intersection = tri.v1 + tri.v1v2 * 0.2;
		Segment seg(intersection - tri.n * 3, intersection + tri.n * 2);
		VectorType cross = tri.n.cross(tri.v1v2);
		segment = Segment(seg.a - cross * 0.01, seg.b - cross * 0.01);
		checkSegTriDistance(SegTriDistanceData(segment, tri, intersection - cross * 0.01, intersection));
	}
}

typedef std::tuple<MockTriangle, MockTriangle, VectorType, VectorType> TriTriDistanceData;
void checkTriTriDistance(const TriTriDistanceData& data)
{
	MockTriangle t0 = std::get<0>(data);
	MockTriangle t1 = std::get<1>(data);
	VectorType expectedT0Point = std::get<2>(data);
	VectorType expectedT1Point = std::get<3>(data);
	double expectedDistance = (expectedT1Point - expectedT0Point).norm();
	double distance;
	VectorType t0Point, t1Point;

	{
		SCOPED_TRACE("Normal Test");
		distance = distanceTriangleTriangle(t0.v0, t0.v1, t0.v2, t1.v0, t1.v1, t1.v2, &t0Point, &t1Point);
		EXPECT_NEAR(expectedDistance, distance, epsilon);
		EXPECT_TRUE(expectedT0Point.isApprox(t0Point));
		EXPECT_TRUE(expectedT1Point.isApprox(t1Point));
	}

// 	{
// 		SCOPED_TRACE("Reversed Triangles");
// 		distance = TriangleTriangleDistance(t1.v0, t1.v1, t1.v2, t0.v0, t0.v1, t0.v2, &t1Point, &t0Point);
// 		EXPECT_NEAR(expectedDistance, distance,epsilon);
// 		EXPECT_TRUE(expectedT0Point.isApprox(t0Point));
// 		EXPECT_TRUE(expectedT1Point.isApprox(t1Point));
// 	}

	{
		SCOPED_TRACE("Shift t0 edges once");
		distance = distanceTriangleTriangle(t0.v1, t0.v2, t0.v0, t1.v0, t1.v1, t1.v2, &t0Point, &t1Point);
		EXPECT_NEAR(expectedDistance, distance, epsilon);
		EXPECT_TRUE(expectedT0Point.isApprox(t0Point));
		EXPECT_TRUE(expectedT1Point.isApprox(t1Point));
	}


	{
		SCOPED_TRACE("Shift t0 edges twice");
		distance = distanceTriangleTriangle(t0.v2, t0.v0, t0.v1, t1.v0, t1.v1, t1.v2, &t0Point, &t1Point);
		EXPECT_NEAR(expectedDistance, distance, epsilon);
		EXPECT_TRUE(expectedT0Point.isApprox(t0Point));
		EXPECT_TRUE(expectedT1Point.isApprox(t1Point));
	}
}

TEST_F(GeometryTest, distanceTriangleTriangle)
{
	MockTriangle t0(VectorType(5, 0, 0), VectorType(0, 2, 2), VectorType(0, -2, -2));
	MockTriangle t1;
	{
		SCOPED_TRACE("vertex t1v0 equal to t0v0");
		t1 = MockTriangle(t0.v0, t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
		checkTriTriDistance(TriTriDistanceData(t1, t0, t0.v0, t0.v0));
	}
	{
		SCOPED_TRACE("vertex t1v0 inside of triangle t0");
		VectorType intersection = t0.pointInTriangle(0.2, 0.2);
		t1 = MockTriangle(intersection, t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
		checkTriTriDistance(TriTriDistanceData(t1, t0, t1.v0, intersection));
	}
	{
		SCOPED_TRACE("vertex t1v0 close to t0v0");
		t1 = MockTriangle(t0.v0 + t0.n, t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
		checkTriTriDistance(TriTriDistanceData(t1, t0, t1.v0, t0.v0));
	}
	{
		SCOPED_TRACE("vertex t1v0 close to the inside of triangle t0");
		VectorType intersection = t0.pointInTriangle(0.2, 0.2);
		t1 = MockTriangle(intersection + t0.n , t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
		checkTriTriDistance(TriTriDistanceData(t1, t0, t1.v0, intersection));
	}
	{
		SCOPED_TRACE("edge t1v0v1 through triangle t0");
		VectorType intersection = t0.pointInTriangle(0.2, 0.2);
		t1 = MockTriangle(intersection + t0.n * 3, t0.v0 - t0.v0v2 * 4 + t0.n, intersection - t0.n * 4);
		checkTriTriDistance(TriTriDistanceData(t1, t0, intersection, intersection));
	}
	{
		SCOPED_TRACE("Triangles parallel");
		t1 = MockTriangle(t0.v0 + tri.n * 3, t0.v1 + tri.n * 3, t0.v2 + tri.n * 3);
		VectorType closest0, closest1;
		double distance = distanceTriangleTriangle(t0.v0, t0.v1, t0.v2, t1.v0, t1.v1, t1.v2, &closest0, &closest1);
		EXPECT_NEAR(3.0, distance, epsilon);
	}
	{
		SCOPED_TRACE("edge t0v0v1 close to t1v0v1");
		VectorType closest0 = t0.v0 + t0.v0v1 * 0.2;
		VectorType shift = t0.n.cross(t0.v0v1.normalized());
		shift.normalize();
		VectorType closest1 = closest0 - shift * 2;
		t1 = MockTriangle(closest1 - tri.n * 2, closest1 + tri.n * 2, closest1 + tri.n - shift * 10);
		checkTriTriDistance(TriTriDistanceData(t1, t0, closest1, closest0));
	}
}

TEST_F(GeometryTest, IntersectionsSegmentBox)
{
	Eigen::AlignedBox<SizeType, 3> box;
	{
		SCOPED_TRACE("No intersection, zero length segment");
		VectorType point1(0.0, 0.0, 0.0);
		VectorType point2(0.0, 0.0, 0.0);
		box.min() = VectorType(1.0 , 1.0, 1.0);
		box.max() = VectorType(5.0 , 5.0, 5.0);
		std::vector<VectorType> intersections;
		intersectionsSegmentBox(point1, point2, box, &intersections);
		EXPECT_EQ(0, intersections.size());
	}

	{
		SCOPED_TRACE("No intersection, zero size box");
		VectorType point1(0.0, 0.0, 0.0);
		VectorType point2(0.0, 5.0, 0.0);
		box.min() = VectorType(1.0 , 1.0, 1.0);
		box.max() = VectorType(1.0 , 1.0, 1.0);
		std::vector<VectorType> intersections;
		intersectionsSegmentBox(point1, point2, box, &intersections);
		EXPECT_EQ(0, intersections.size());
	}

	{
		SCOPED_TRACE("No Intersection, parallel and beyond corners");
		VectorType point1(-0.0, 0.0, -0.0);
		VectorType point2(0.0, 5.0, -0.0);
		box.min() = VectorType(1.0 , 1.0, 1.0);
		box.max() = VectorType(5.0 , 5.0, 5.0);
		std::vector<VectorType> intersections;
		intersectionsSegmentBox(point1, point2, box, &intersections);
		EXPECT_EQ(0, intersections.size());
	}

	{
		SCOPED_TRACE("Entering box, but not leaving");
		VectorType point1(2.0, 2.0, 0.0);
		VectorType point2(3.0, 3.0, 2.0);
		box.min() = VectorType(1.0 , 1.0, 1.0);
		box.max() = VectorType(5.0 , 5.0, 5.0);
		std::vector<VectorType> intersections;
		intersectionsSegmentBox(point1, point2, box, &intersections);
		EXPECT_EQ(1, intersections.size());
		EXPECT_TRUE(intersections[0].isApprox(VectorType(2.5, 2.5, 1.0)));
	}

	{
		SCOPED_TRACE("Entering and exiting box, through box corners");
		VectorType point1(0.0, 0.0, 0.0);
		VectorType point2(6.0, 6.0, 6.0);
		box.min() = VectorType(1.0 , 1.0, 1.0);
		box.max() = VectorType(5.0 , 5.0, 5.0);
		std::vector<VectorType> intersections;
		intersectionsSegmentBox(point1, point2, box, &intersections);
		EXPECT_EQ(2, intersections.size());
		EXPECT_TRUE(intersections[0].isApprox(box.min()) || intersections[0].isApprox(box.max()));
		EXPECT_TRUE(intersections[1].isApprox(box.min()) || intersections[1].isApprox(box.max()));
	}
}

TEST_F(GeometryTest, DoesIntersectBoxCapsule)
{
	typedef Eigen::AlignedBox<SizeType, 3> BoxType;
	{
		SCOPED_TRACE("No intersection");
		VectorType bottom(-5.0, 5.0, 0.0);
		VectorType top(5.0, 5.0, 0.0);
		double radius = 1.0;
		BoxType box(VectorType(-1.0 , -1.0, -1.0), VectorType(1.0 , 1.0, 1.0));
		EXPECT_FALSE(doesIntersectBoxCapsule(bottom, top, radius, box));
	}
	{
		SCOPED_TRACE("Intersection, capsule in middle of box");
		VectorType bottom(-5.0, -5.0, -5.0);
		VectorType top(5.0, 5.0, 5.0);
		double radius = 10.0;
		BoxType box(VectorType(-1.0 , -1.0, -1.0), VectorType(1.0 , 1.0, 1.0));
		EXPECT_TRUE(doesIntersectBoxCapsule(bottom, top, radius, box));
	}
	{
		SCOPED_TRACE("No Intersection, box not centered");
		VectorType bottom(-5.0, -5.0, -5.0);
		VectorType top(5.0, 5.0, 5.0);
		double radius = 1.0;
		BoxType box(VectorType(1.0 , 1.0, -1.0), VectorType(2.0 , 2.0, -2.0));
		EXPECT_FALSE(doesIntersectBoxCapsule(bottom, top, radius, box));
	}
	{
		SCOPED_TRACE("Intersection, box not centered");
		VectorType bottom(-5.0, -5.0, -5.0);
		VectorType top(5.0, 5.0, 5.0);
		double radius = 1.0;
		BoxType box(VectorType(0.0 , 0.0, 0.0), VectorType(1.0 , 1.0, 1.0));
		EXPECT_TRUE(doesIntersectBoxCapsule(bottom, top, radius, box));
	}
	{
		SCOPED_TRACE("No intersection, capsule along edge");
		VectorType bottom(2.0, -2.0, 2.0);
		VectorType top(2.0, 2.0, 2.0);
		double radius = sqrt(2.0) - 1.0;
		BoxType box(VectorType(-1.0 , -1.0, -1.0), VectorType(1.0 , 1.0, 1.0));
		EXPECT_FALSE(doesIntersectBoxCapsule(bottom, top, radius, box));
	}
	{
		SCOPED_TRACE("Intersection, capsule along edge");
		VectorType bottom(2.0, -2.0, 2.0);
		VectorType top(2.0, 2.0, 2.0);
		double radius = sqrt(2.0) + 0.1;
		BoxType box(VectorType(-1.0 , -1.0, -1.0), VectorType(1.0 , 1.0, 1.0));
		EXPECT_TRUE(doesIntersectBoxCapsule(bottom, top, radius, box));
	}
	{
		SCOPED_TRACE("No Intersection, capsule at corner");
		VectorType bottom(2.0, 3.0, 1.0);
		VectorType top(2.0, 1.0, 3.0);
		double radius = sqrt(3.0) - 0.1;
		BoxType box(VectorType(-1.0 , -1.0, -1.0), VectorType(1.0 , 1.0, 1.0));
		EXPECT_FALSE(doesIntersectBoxCapsule(bottom, top, radius, box));
	}
	{
		SCOPED_TRACE("Intersection, capsule at corner");
		VectorType bottom(2.0, 3.0, 1.0);
		VectorType top(2.0, 1.0, 3.0);
		double radius = sqrt(3.0) + 0.1;
		BoxType box(VectorType(-1.0 , -1.0, -1.0), VectorType(1.0 , 1.0, 1.0));
		EXPECT_TRUE(doesIntersectBoxCapsule(bottom, top, radius, box));
	}
}

}; // namespace Math
}; // namespace SurgSim

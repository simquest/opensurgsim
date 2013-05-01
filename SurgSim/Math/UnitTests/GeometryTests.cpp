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

/** @file
 * Tests for the Intersections.cpp functions.
 */

#include <gtest/gtest.h>
#include <numeric>
#include <cmath>

#include <SurgSim/Math/Geometry.h>
#include <boost/math/special_functions/fpclassify.hpp>


using namespace SurgSim::Math;


typedef double SizeType;
typedef Eigen::Matrix<SizeType, 3, 1, Eigen::DontAlign> VectorType;

::std::ostream& operator <<(std::ostream& stream, const VectorType& vector)
{
	stream << "(" << vector[0] << ", " << vector[1] << ", " << vector[2] << ")";
	return stream;
}

bool near(double val1, double val2, double abs_error)
{
	const double diff = fabs(val1 - val2);
	if (diff <= abs_error) return true;
	else return false;
}

::testing::AssertionResult eigenEqual(const VectorType& expected, const VectorType& actual)
{
	double error = 1e-4;
	if (near(expected[0], actual[0],error) && near(expected[1], actual[1], error) &&
		near(expected[2], actual[2],error))
	{
		return ::testing::AssertionSuccess();
	}
	else 
	{
		return ::testing::AssertionFailure() << "Eigen Matrices not the same " << std::endl <<
				"expected: " << expected << std::endl << "actual: " << actual <<std::endl;
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

	Segment() {};
	Segment (const VectorType& pointA, const VectorType& pointB) :
		a(pointA), b(pointB), ab(pointB-pointA) {}
	~Segment() {}
	/// Point on the line that the segment is on, s =< 1 and s >= 0 will give you a point on the segment
	VectorType pointOnLine(double s)
	{
		return a+ab*s;
	}
};

class Triangle
{
public:
	VectorType v0;
	VectorType v1;
	VectorType v2;

	VectorType v0v1;
	VectorType v0v2;
	VectorType v1v2;

	VectorType n;

	Triangle() {};
	Triangle(VectorType vertex0, VectorType vertex1, VectorType vertex2) :
		v0(vertex0), v1(vertex1), v2(vertex2), v0v1(vertex1-vertex0), v0v2(vertex2-vertex0), v1v2(vertex2-vertex1)
		{
			n = v0v1.cross(v0v2);
			n.normalize();
		}

	VectorType pointInTriangle(SizeType a, SizeType b)
	{
		return v0 + a*v0v1 + b*v0v2;
	}
};

class GeometryTest : public ::testing::Test
{
protected:
	virtual void SetUp() 
	{
		plainPoint = VectorType(45,20,10);
		plainSegment = Segment(VectorType(1.1,2.2,3.3),VectorType(6.6,5.5,4.4));
		plainNormal = plainSegment.ab.cross(plainPoint); // Normal to segment
		plainNormal.normalize();

		degenerateSegment.a = plainSegment.a;
		degenerateSegment.b = degenerateSegment.a + (plainSegment.ab)*1e-9;
		degenerateSegment.ab = degenerateSegment.b - degenerateSegment.a;
		epsilon = 1e-10;

		plainLine = Segment(VectorType(-10.0,10,10), VectorType(10.0,10.0,10.0));
		parallelLine = Segment(VectorType(-100.0,5.0,5.0), VectorType(-90.0,5.0,5.0));
		intersectingLine = Segment(VectorType(0,0,0),VectorType(20,20,20));
		nonIntersectingLine= Segment(VectorType(5,5,-5),VectorType(5,5,5));

		tri = Triangle(VectorType(5,0,0), VectorType(0,-5,-5), VectorType(0,5,5));
	}

	virtual void TearDown()
	{
	}
	VectorType plainPoint;
	Vector3d plainNormal;
	Segment plainSegment;
	Segment degenerateSegment;
	SizeType epsilon;

	Segment plainLine;
	Segment parallelLine;
	Segment intersectingLine;
	Segment nonIntersectingLine;

	Triangle tri;
};


TEST_F(GeometryTest, BaryCentricWithNormal)
{
	// Order of Points is v0,v1,v2
	//Check Edges first
	VectorType outputPoint;
	EXPECT_TRUE(BaryCentricCoordinates(tri.v0, tri.v0,tri.v1,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(1,0,0),outputPoint));

	EXPECT_TRUE(BaryCentricCoordinates(tri.v1, tri.v0,tri.v1,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0,1,0),outputPoint));

	EXPECT_TRUE(BaryCentricCoordinates(tri.v2, tri.v0,tri.v1,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0,0,1),outputPoint));

	// Halfway points
	EXPECT_TRUE(BaryCentricCoordinates<double>(tri.pointInTriangle(0.5,0), 
		tri.v0,tri.v1,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.5,0.5,0),outputPoint));

	EXPECT_TRUE(BaryCentricCoordinates<double>(tri.pointInTriangle(0,0.5), 
		tri.v0,tri.v1,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.5,0.0,0.5),outputPoint));

	// Center Point
	VectorType inputPoint;
	inputPoint = (tri.v0 + tri.v1 + tri.v2) / 3;
	EXPECT_TRUE(BaryCentricCoordinates(inputPoint, tri.v0,tri.v1,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(1.0/3.0,1.0/3.0,1.0/3.0),outputPoint));

	// random Point
	inputPoint = tri.v0*0.2 + tri.v1*0.25 + tri.v2*0.55;
	EXPECT_TRUE(BaryCentricCoordinates(inputPoint, tri.v0,tri.v1,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.2,0.25,0.55),outputPoint));

	// Degenerate
	EXPECT_FALSE(BaryCentricCoordinates(inputPoint, tri.v1,tri.v1,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));

	EXPECT_FALSE(BaryCentricCoordinates(inputPoint, tri.v0,tri.v0,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));

	EXPECT_FALSE(BaryCentricCoordinates(inputPoint, tri.v2,tri.v1,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));
}

TEST_F(GeometryTest, BaryCentricWithoutNormal)
{
	// Order of Points is v0,v1,v2
	//Check Edges first
	VectorType outputPoint;
	EXPECT_TRUE(BaryCentricCoordinates(tri.v0, tri.v0,tri.v1,tri.v2, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(1,0,0),outputPoint));

	EXPECT_TRUE(BaryCentricCoordinates(tri.v1, tri.v0,tri.v1,tri.v2, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0,1,0),outputPoint));

	EXPECT_TRUE(BaryCentricCoordinates(tri.v2, tri.v0,tri.v1,tri.v2, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0,0,1),outputPoint));

	// Halfway points
	EXPECT_TRUE(BaryCentricCoordinates<double>(tri.pointInTriangle(0.5,0), 
		tri.v0,tri.v1,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.5,0.5,0),outputPoint));

	EXPECT_TRUE(BaryCentricCoordinates<double>(tri.pointInTriangle(0,0.5), 
		tri.v0,tri.v1,tri.v2,tri.n,&outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.5,0.0,0.5),outputPoint));

	// Center Point
	VectorType inputPoint;
	inputPoint = (tri.v0 + tri.v1 + tri.v2) / 3;
	EXPECT_TRUE(BaryCentricCoordinates(inputPoint, tri.v0,tri.v1,tri.v2, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(1.0/3.0,1.0/3.0,1.0/3.0),outputPoint));

	// random Point
	inputPoint = tri.v0*0.2 + tri.v1*0.25 + tri.v2*0.55;
	EXPECT_TRUE(BaryCentricCoordinates(inputPoint, tri.v0,tri.v1,tri.v2, &outputPoint));
	EXPECT_TRUE(eigenEqual(VectorType(0.2,0.25,0.55),outputPoint));

	// Degenerate
	EXPECT_FALSE(BaryCentricCoordinates(inputPoint, tri.v1,tri.v1,tri.v2, &outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));

	EXPECT_FALSE(BaryCentricCoordinates(inputPoint, tri.v0,tri.v0,tri.v2, &outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));

	EXPECT_FALSE(BaryCentricCoordinates(inputPoint, tri.v2,tri.v1,tri.v2, &outputPoint));
	EXPECT_TRUE(eigenAllNan(outputPoint));
}

TEST_F(GeometryTest, DistancePointLine)
{
	SizeType distance;
	Vector3d result;

	// Trivial point lies on the line
	distance = PointLineDistance<SizeType>(plainSegment.pointOnLine(0.5), plainSegment.a, plainSegment.b, &result);
	EXPECT_DOUBLE_EQ(0.0,distance);
	EXPECT_EQ(plainSegment.pointOnLine(0.5),result);

	// Point is away from the line
	Vector3d offLinePoint = plainSegment.a + (plainNormal * 1.5);
	distance = PointLineDistance(offLinePoint, plainSegment.a, plainSegment.b, &result);
	EXPECT_DOUBLE_EQ(1.5, distance);
	EXPECT_EQ(plainSegment.a, result);

	// Degenerate line, just do plain distance
	offLinePoint = plainSegment.a + plainNormal * 1.5;
	distance = PointLineDistance(offLinePoint, plainSegment.a, plainSegment.a, &result);
	EXPECT_DOUBLE_EQ(1.5, distance);
	EXPECT_EQ(plainSegment.a, result);
}

TEST_F(GeometryTest, DistancePointSegment)
{
	SizeType distance;
	Vector3d result;

	// Trivial point lies on the line
	distance = PointSegDistance(plainSegment.pointOnLine(0.5), plainSegment.a, plainSegment.b, &result);
	EXPECT_DOUBLE_EQ(0.0,distance);
	EXPECT_EQ(plainSegment.pointOnLine(0.5),result);

	// Point On the line but outside the segment
	VectorType point = plainSegment.pointOnLine(1.5);
	distance = PointSegDistance(point, plainSegment.a, plainSegment.b, &result);
	EXPECT_DOUBLE_EQ((plainSegment.ab.norm()*0.5),distance);
	EXPECT_TRUE(eigenEqual(plainSegment.b,result));

	// Point projection is on the segment
	VectorType resultPoint = plainSegment.a + plainSegment.ab * 0.25;
	VectorType offLinePoint = resultPoint + (plainNormal * 1.5);
	distance = PointSegDistance(offLinePoint, plainSegment.a, plainSegment.b, &result);
	EXPECT_DOUBLE_EQ(1.5, distance);
	EXPECT_TRUE(eigenEqual(resultPoint, result));

	// Point projection is away from the segment, distance is to the closest segment point 
	resultPoint = plainSegment.a;
	offLinePoint = plainSegment.a - plainSegment.ab + (plainNormal * 1.5);
	distance = PointSegDistance(offLinePoint, plainSegment.a, plainSegment.b, &result);
	EXPECT_DOUBLE_EQ((offLinePoint - resultPoint).norm(), distance);
	EXPECT_TRUE(eigenEqual(resultPoint, result));

	// Degenerated Segment
	distance = PointSegDistance(offLinePoint, plainSegment.a, plainSegment.a, &result);
	EXPECT_DOUBLE_EQ((offLinePoint - plainSegment.a).norm(), distance);
	EXPECT_TRUE(eigenEqual(plainSegment.a, result));
}

TEST_F(GeometryTest, DistanceLineLine)
{
	SizeType distance;
	VectorType p0, p1;

	// Trivial the same line compared against itself
	distance = LineLineDistance(plainSegment.a, plainSegment.b, plainSegment.a, plainSegment.b, &p0, &p1);
	EXPECT_DOUBLE_EQ(0.0, distance);

	// Parallel Line
	Segment parallel = Segment(plainSegment.a + plainNormal*2, plainSegment.b + plainNormal*2);
	distance = LineLineDistance(plainSegment.a, plainSegment.b, parallel.a, parallel.b, &p0, &p1);
	EXPECT_DOUBLE_EQ(2.0, distance);

	// Intersecting Line
	distance = LineLineDistance(plainLine.a, plainLine.b, intersectingLine.a, intersectingLine.b, &p0, &p1);
	EXPECT_DOUBLE_EQ(0.0, distance);
	EXPECT_TRUE(eigenEqual(plainLine.b, p0));
	EXPECT_TRUE(eigenEqual(plainLine.b, p1));

	// Non Intersecting Line, don't know a better way to design this case besides reimplementing line/line distance
	Segment line0(VectorType(0,-5,0), VectorType(0,5,0));
	Segment line1(VectorType(-5,5,5), VectorType(5,5,5));

	distance = LineLineDistance(line0.a, line0.b, line1.a, line1.b, &p0, &p1);
	EXPECT_DOUBLE_EQ(5, distance);
	EXPECT_TRUE(eigenEqual(VectorType(0,5,0), p0));
	EXPECT_TRUE(eigenEqual(VectorType(0,5,5), p1));

	// Degenerate Cases
	// Both Lines Degenerate
	distance = LineLineDistance(plainSegment.a, plainSegment.a, plainSegment.b, plainSegment.b, &p0, &p1);
	EXPECT_DOUBLE_EQ(plainSegment.ab.norm(),distance);
	EXPECT_TRUE(eigenEqual(plainSegment.a,p0));
	EXPECT_TRUE(eigenEqual(plainSegment.b,p1));

	// First Line Denegenerate
	VectorType offLinePoint = plainSegment.a + plainSegment.ab*0.5 + plainNormal * 1.5;
	distance = LineLineDistance(offLinePoint, offLinePoint, plainSegment.a, plainSegment.b, &p0, &p1);
	EXPECT_DOUBLE_EQ(1.5,distance);
	EXPECT_TRUE(eigenEqual(offLinePoint,p0));
	EXPECT_TRUE(eigenEqual(plainSegment.a + plainSegment.ab*0.5, p1));

	// Second Line Degenerate
	distance = LineLineDistance(plainSegment.a, plainSegment.b, offLinePoint, offLinePoint, &p0, &p1);
	EXPECT_DOUBLE_EQ(1.5,distance);
	EXPECT_TRUE(eigenEqual(offLinePoint,p1));
	EXPECT_TRUE(eigenEqual(plainSegment.a + plainSegment.ab*0.5, p0));	
}



struct SegmentData {
	Segment segment0;
	Segment segment1;
	VectorType p0;
	VectorType p1;
	SegmentData () {};
	SegmentData (Segment seg0, Segment seg1, VectorType vec0, VectorType vec1) :
		segment0(seg0), segment1(seg1), p0(vec0), p1(vec1) {};
};


void testSegmentDistance(const SegmentData& segmentData, std::string info, int i)
{
	SizeType distance;
	VectorType p0, p1;

	// The expected distance should be the distance between the two points that were 
	// reported as being the closes ones
	SizeType expectedDistance = (segmentData.p1 - segmentData.p0).norm();

	distance = SegSegDistance(segmentData.segment0.a, segmentData.segment0.b, segmentData.segment1.a, segmentData.segment1.b, &p0, &p1, 1e-8);
	EXPECT_NEAR(expectedDistance, distance, 1e-8) << "for " << info << " at index " << i;
	EXPECT_TRUE(eigenEqual(segmentData.p0, p0)) << "for " << info << " at index " << i;
	EXPECT_TRUE(eigenEqual(segmentData.p1, p1)) << "for " << info << " at index " << i;

	distance = SegSegDistance(segmentData.segment1.a, segmentData.segment1.b, segmentData.segment0.a, segmentData.segment0.b, &p0, &p1, 1e-8);
	EXPECT_NEAR(expectedDistance, distance,1e-8) << "for " << info << " at index " << i;
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

	distance = SegSegDistance(plainSegment.a, plainSegment.b, otherSegment.a, otherSegment.b, &p0, &p1, epsilon);
	EXPECT_NEAR(0.0, distance,epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint,p0));
	EXPECT_TRUE(eigenEqual(closestPoint,p1));

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
	otherSegment = Segment(closestPoint+plainNormal, closestPoint - plainNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, closestPoint));

	// <1> segment not straddling, the correct points on the edges of the segments should get picked
	otherSegment = Segment(closestPoint+plainNormal, closestPoint + plainNormal*2);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, otherSegment.a));

	// <2> segment not straddling, reverse the order of the points, reverse the side where the other segments falls
	otherSegment = Segment(closestPoint-plainNormal*2, closestPoint - plainNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, otherSegment.b));

	// Go to the other side of the segment
	closestPoint = plainSegment.pointOnLine(-0.5);
	// <3> Straddling, there is actual an intersection
	otherSegment = Segment(closestPoint+plainNormal, closestPoint - plainNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, closestPoint));

	// <4> segment not straddling, the correct points on the edges of the segments should get picked
	otherSegment = Segment(closestPoint+plainNormal, closestPoint + plainNormal*2);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, otherSegment.a));


	// <5> segment not straddling, reverse the order of the points, reverse the side where the other segments falls
	otherSegment = Segment(closestPoint-plainNormal*2, closestPoint - plainNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, otherSegment.b));


	// Repeat the same sequence for the segments as they are not touching
	VectorType otherNormal = plainSegment.ab.cross(plainNormal);

	// <6> segment projections intersect
	closestPoint = plainSegment.pointOnLine(0.5) + plainNormal*3;
	otherSegment = Segment(closestPoint + otherNormal, closestPoint - otherNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.pointOnLine(0.5), closestPoint));

	// <7> go past the end of the segment but straddle the line (T intersection)
	closestPoint = plainSegment.pointOnLine(1.5) + plainNormal*3;
	otherSegment = Segment(closestPoint + otherNormal, closestPoint - otherNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, closestPoint));
	
	// <8> go past the end of the segment not straddling the line anymore
	otherSegment = Segment(closestPoint + otherNormal, closestPoint + otherNormal*2);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, otherSegment.a));

	// <9> go past the end of the on the other side, switching up endpoints
	otherSegment = Segment(closestPoint - otherNormal*2, closestPoint - otherNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, otherSegment.b));

	// Repeat for the other end of the base segment
	// <10> go past the end of the segment but straddle the line (T intersection)
	closestPoint = plainSegment.pointOnLine(-2.0) + plainNormal*3;
	otherSegment = Segment(closestPoint + otherNormal, closestPoint - otherNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, closestPoint));

	// <11> go past the end of the segment not straddling the line anymore
	otherSegment = Segment(closestPoint + otherNormal, closestPoint + otherNormal*2);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, otherSegment.a));

	// <12> go past the end of the on the other side, switching up endpoints
	otherSegment = Segment(closestPoint - otherNormal*2, closestPoint - otherNormal);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, otherSegment.b));

	for(size_t i = 0; i < segments.size(); ++i)
	{
		testSegmentDistance(segments[i],"basic cases", i);
	}

	// Parallel Segments
	closestPoint = plainSegment.a;
	otherSegment = Segment(plainSegment.a + plainNormal *4, plainSegment.b + plainNormal*4);
	distance = SegSegDistance(plainSegment.a, plainSegment.b, otherSegment.a, otherSegment.b, &p0, &p1, epsilon);
	EXPECT_NEAR(4.0, distance, epsilon);
	// What should the points be here ? 
	
	segments.clear();

	// The closest points are some assumptions, it looks like the algorithm is slanted towards 
	// <0> the begining points of the segments for this
	closestPoint = plainSegment.pointOnLine(0.5);
	otherSegment = Segment(closestPoint + plainNormal *4, closestPoint + plainNormal*8);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.pointOnLine(0.5), otherSegment.a));

	// <1> Move past the end of the segment on the far end
	closestPoint = plainSegment.pointOnLine(1.5);
	otherSegment = Segment(closestPoint + plainNormal *4, closestPoint + plainNormal*8);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.b, otherSegment.a));

	// <2> Move past the end of the segment on the near end
	closestPoint = plainSegment.pointOnLine(-2.0);
	otherSegment = Segment(closestPoint - plainNormal * 8, closestPoint - plainNormal * 4);
	segments.push_back(SegmentData(plainSegment, otherSegment, plainSegment.a, otherSegment.b));


	// Degenerate cases delegate to PointSegDistance, just some spotChecks
	// <3> On the segment
	closestPoint = plainSegment.pointOnLine(0.5);
	otherSegment = Segment(closestPoint, closestPoint);
	segments.push_back(SegmentData(plainSegment, otherSegment,closestPoint, closestPoint));

	// <4> off the segment
	closestPoint = plainSegment.pointOnLine(1.5) + plainNormal*4 + otherNormal * 10;
	otherSegment = Segment(closestPoint, closestPoint);
	segments.push_back(SegmentData(plainSegment, otherSegment,plainSegment.b, closestPoint));


	for(size_t i = 0; i < segments.size(); ++i)
	{
		testSegmentDistance(segments[i],"other cases",i);
	}
}
	
TEST_F(GeometryTest, DistancePointTriangle)
{
	double distance;
	VectorType closestPoint;
	VectorType result;
	VectorType inputPoint;

	// Trivial, point on triangle
	inputPoint = VectorType(0,0,0);
	distance = PointTriDistance(inputPoint,tri.v0,tri.v1,tri.v2,&result);
	EXPECT_NEAR(0.0, distance, epsilon);
	EXPECT_TRUE(eigenEqual(inputPoint,result));

	distance = PointTriDistance(tri.v1,tri.v0,tri.v1,tri.v2,&result);
	EXPECT_NEAR(0.0, distance, epsilon);
	EXPECT_TRUE(eigenEqual(tri.v1,result));

	// Closest Point is inside Triangle
	closestPoint = tri.v0 + tri.v0v1*0.3 + tri.v0v2*0.7;
	inputPoint = closestPoint + tri.n * 2.5;
	distance = PointTriDistance(inputPoint,tri.v0,tri.v1,tri.v2,&result);
	EXPECT_NEAR(2.5, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint,result));

	// other side
	inputPoint = closestPoint - tri.n * 3.5;
	distance = PointTriDistance(inputPoint,tri.v0,tri.v1,tri.v2,&result);
	EXPECT_NEAR(3.5, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint,result));

	// Test the Point close to a triangle Edge cases 
	// Point closest to edge v0v1
	double expectedDistance;
	inputPoint = tri.v0 +  tri.v0v1*0.5 - tri.v0v2 + tri.n;
	distance = PointTriDistance(inputPoint,tri.v0,tri.v1,tri.v2,&result);
	expectedDistance = PointSegDistance(inputPoint, tri.v0,tri.v1, &closestPoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint,result));

	// Point closest to edge v0v2
	inputPoint = tri.v0 + tri.v0v2*0.3 - tri.v0v1 +tri.n*2;
	distance = PointTriDistance(inputPoint,tri.v0,tri.v1,tri.v2,&result);
	expectedDistance = PointSegDistance(inputPoint, tri.v0,tri.v2, &closestPoint);	
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint,result));

	// Point closest to edge v1v2
	inputPoint = tri.v1 + (tri.v2-tri.v1)*.75 + tri.v0v1*0.2 + tri.n;
	distance = PointTriDistance(inputPoint,tri.v0,tri.v1,tri.v2,&result);
	expectedDistance = PointSegDistance(inputPoint, tri.v1,tri.v2, &closestPoint);	
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint,result));

	// Point closest to point v0
	inputPoint = tri.v0 - tri.v0v1 - tri.v0v2*0.5 - tri.n;
	distance = PointTriDistance(inputPoint,tri.v0,tri.v1,tri.v2,&result);
	expectedDistance = (tri.v0 - inputPoint).norm();	
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(tri.v0,result));

	// Point closest to point v1
	inputPoint = tri.v1 + tri.v0v1 + (tri.v1 - tri.v2) * 2.0 - tri.n*2.0;
	distance = PointTriDistance(inputPoint,tri.v0,tri.v1,tri.v2,&result);
	expectedDistance = (tri.v1 - inputPoint).norm();	
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(tri.v1,result));

	// Point closest to point v2
	inputPoint = tri.v2 + tri.v0v2 + (tri.v2 - tri.v1) * 3.0 - tri.n*1.5;
	distance = PointTriDistance(inputPoint,tri.v0,tri.v1,tri.v2,&result);
	expectedDistance = (tri.v2 - inputPoint).norm();	
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(tri.v2,result));

	// Degenerate Edges
	// Edge v0v1
	distance = PointTriDistance<double>(inputPoint, tri.v0, tri.v0 + tri.v0v1*epsilon*0.01,tri.v2, &result);
	expectedDistance = PointSegDistance(inputPoint,tri.v0,tri.v2, &closestPoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint,result));

	// Edge v0v2
	distance = PointTriDistance<double>(inputPoint, tri.v2 - tri.v0v2*epsilon*0.01 , tri.v1 ,tri.v2, &result);
	expectedDistance = PointSegDistance(inputPoint,tri.v1,tri.v2, &closestPoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint,result));
	
	// Edge v1v2
	distance = PointTriDistance(inputPoint, tri.v0, tri.v1 ,tri.v1, &result);
	expectedDistance = PointSegDistance(inputPoint,tri.v1,tri.v0, &closestPoint);
	EXPECT_NEAR(expectedDistance, distance, epsilon);
	EXPECT_TRUE(eigenEqual(closestPoint,result));
}

TEST_F(GeometryTest, PointInsideTriangleWithNormal)
{
	EXPECT_TRUE(PointInsideTriangle(tri.v0, tri.v0,tri.v1,tri.v2,tri.n));
	EXPECT_TRUE(PointInsideTriangle(tri.v1, tri.v0,tri.v1,tri.v2,tri.n));
	EXPECT_TRUE(PointInsideTriangle(tri.v2, tri.v0,tri.v1,tri.v2,tri.n));

	VectorType inputPoint = tri.v0 + tri.v0v1*0.2 ;
	EXPECT_TRUE(PointInsideTriangle(inputPoint, tri.v0,tri.v1,tri.v2,tri.n));
	inputPoint += tri.v0v2*0.5;
	EXPECT_TRUE(PointInsideTriangle(inputPoint, tri.v0,tri.v1,tri.v2,tri.n));
	
	inputPoint = tri.v0 + tri.v0v1*1.5;
	EXPECT_FALSE(PointInsideTriangle(inputPoint, tri.v0,tri.v1,tri.v2,tri.n));
	EXPECT_FALSE(PointInsideTriangle(inputPoint, tri.v1,tri.v1,tri.v2,tri.n));

	inputPoint = tri.v0 + tri.v0v2*2 + tri.v0v1 * 2;
	EXPECT_FALSE(PointInsideTriangle(inputPoint, tri.v0,tri.v1,tri.v2,tri.n));

}

TEST_F(GeometryTest, PointInsideTriangleWithoutNormal)
{
	EXPECT_TRUE(PointInsideTriangle(tri.v0, tri.v0,tri.v1,tri.v2));
	EXPECT_TRUE(PointInsideTriangle(tri.v1, tri.v0,tri.v1,tri.v2));
	EXPECT_TRUE(PointInsideTriangle(tri.v2, tri.v0,tri.v1,tri.v2));

	VectorType inputPoint = tri.v0 + tri.v0v1*0.2 ;
	EXPECT_TRUE(PointInsideTriangle(inputPoint, tri.v0,tri.v1,tri.v2));
	inputPoint += tri.v0v2*0.5;
	EXPECT_TRUE(PointInsideTriangle(inputPoint, tri.v0,tri.v1,tri.v2));

	inputPoint = tri.v0 + tri.v0v1*1.5;
	EXPECT_FALSE(PointInsideTriangle(inputPoint, tri.v0,tri.v1,tri.v2));
	EXPECT_FALSE(PointInsideTriangle(inputPoint, tri.v1,tri.v1,tri.v2));

	inputPoint = tri.v0 + tri.v0v2*2 + tri.v0v1 * 2;
	EXPECT_FALSE(PointInsideTriangle(inputPoint, tri.v0,tri.v1,tri.v2));
}

TEST_F(GeometryTest, SegmentTriangleIntersection)
{
	VectorType closestPoint;
	VectorType intersectionPoint = tri.pointInTriangle(0.2,0.7);
	Segment intersecting(intersectionPoint - tri.n*2, intersectionPoint + tri.n*2);

	double distance;

	distance = SegTriCollide(intersecting.a, intersecting.b, tri.v0, tri.v1, tri.v2, tri.n,&closestPoint);
	EXPECT_NEAR(-2.0,distance,epsilon);
	EXPECT_TRUE(eigenEqual(intersectionPoint, closestPoint));

	intersecting.a = intersectionPoint + tri.n*4;
	distance = SegTriCollide(intersecting.a, intersecting.b, tri.v0, tri.v1, tri.v2, tri.n,&closestPoint);
	EXPECT_NEAR(1,distance,epsilon);
	EXPECT_TRUE(eigenEqual(intersectionPoint, closestPoint));

}


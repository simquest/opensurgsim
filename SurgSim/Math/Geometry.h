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

#ifndef SURGSIM_MATH_GEOMETRY_H
#define SURGSIM_MATH_GEOMETRY_H

#include <boost/container/static_vector.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Vector.h"

/// \file Geometry.h a collection of functions that calculation geometric properties of various basic geometric shapes.
/// 	  Point, LineSegment, Plane, Triangle. All functions are templated for the accuracy of the calculation
/// 	  (float/double). There are also three kinds of epsilon defined that are used on a case by case basis.
/// 	  In general all function here will return a floating point or boolean value and take a series of output
/// 	  parameters. When those outputs cannot be calculated their values will be set to NAN.
/// 	  This functions are meant as a basic layer that will be wrapped with calls from structures mainting more
/// 	  state information about the primitives they are handling.
/// 	  As a convention we are using a plane equation in the form nx + d = 0
/// \note HS-2013-may-07 Even though some of the names in this file do not agree with the coding standards in
/// 	  regard to the use of verbs for functions it was determined that other phrasing would not necessarily
/// 	  improve the readability or expressiveness of the function names.

namespace SurgSim
{
namespace Math
{

namespace Geometry
{

/// Used as epsilon for general distance calculations
static const double DistanceEpsilon = 1e-10;

/// Used as epsilon for general distance calculations with squared distances
static const double SquaredDistanceEpsilon = 1e-10;

/// Epsilon used in angular comparisons
static const double AngularEpsilon = 1e-10;

/// Used as epsilon for scalar comparisons
static const double ScalarEpsilon = 1e-10;

}

/// Calculate the barycentric coordinates of a point with respect to a line segment.
/// \tparam T Floating point type of the calculation, can usually be inferred.
/// \tparam MOpt Eigen Matrix options, can usually be inferred.
/// \param pt Vertex of the point.
/// \param sv0, sv1 Vertices of the line segment.
/// \param [out] coordinates Barycentric coordinates.
/// \return bool true on success, false if two or more if the line segment is considered degenerate
/// \note The point need not be on the line segment, in which case, the barycentric coordinate of the projection
/// is calculated.
template <class T, int MOpt> inline
bool barycentricCoordinates(const Eigen::Matrix<T, 3, 1, MOpt>& pt,
							const Eigen::Matrix<T, 3, 1, MOpt>& sv0,
							const Eigen::Matrix<T, 3, 1, MOpt>& sv1,
							Eigen::Matrix<T, 2, 1, MOpt>* coordinates)
{
	const Eigen::Matrix<T, 3, 1, MOpt> line = sv1 - sv0;
	const T length2 = line.squaredNorm();
	if (length2 < Geometry::SquaredDistanceEpsilon)
	{
		coordinates->setConstant((std::numeric_limits<double>::quiet_NaN()));
		return false;
	}
	(*coordinates)[1] = (pt - sv0).dot(line) / length2;
	(*coordinates)[0] = static_cast<T>(1) - (*coordinates)[1];
	return true;
}

/// Calculate the barycentric coordinates of a point with respect to a triangle.
/// \pre The normal must be unit length
/// \pre The triangle vertices must be in counter clockwise order in respect to the normal
/// \tparam T Floating point type of the calculation, can usually be inferred.
/// \tparam MOpt Eigen Matrix options, can usually be inferred.
/// \param pt Vertex of the point.
/// \param tv0, tv1, tv2 Vertices of the triangle in counter clockwise order in respect to the normal.
/// \param tn Normal of the triangle (yes must be of norm 1 and a,b,c CCW).
/// \param [out] coordinates Barycentric coordinates.
/// \return bool true on success, false if two or more if the triangle is considered degenerate
template <class T, int MOpt> inline
bool barycentricCoordinates(const Eigen::Matrix<T, 3, 1, MOpt>& pt,
							const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
							const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
							const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
							const Eigen::Matrix<T, 3, 1, MOpt>& tn,
							Eigen::Matrix<T, 3, 1, MOpt>* coordinates)
{
	const T signedTriAreaX2 = ((tv1 - tv0).cross(tv2 - tv0)).dot(tn);
	if (signedTriAreaX2 < Geometry::SquaredDistanceEpsilon)
	{
		// SQ_ASSERT_WARNING(false, "Cannot compute barycentric coords (degenetrate triangle), assigning center");
		coordinates->setConstant((std::numeric_limits<double>::quiet_NaN()));
		return false;
	}
	(*coordinates)[0] = ((tv1 - pt).cross(tv2 - pt)).dot(tn) / signedTriAreaX2;
	(*coordinates)[1] = ((tv2 - pt).cross(tv0 - pt)).dot(tn) / signedTriAreaX2;
	(*coordinates)[2] = 1 - (*coordinates)[0] - (*coordinates)[1];
	return true;
}

/// Calculate the barycentric coordinates of a point with respect to a triangle.
/// Please note that each time you use this call the normal of the triangle will be
/// calculated, if you convert more than one coordinate against this triangle, precalculate
/// the normal and use the other version of this function
/// \tparam T Floating point type of the calculation, can usually be inferred.
/// \tparam MOpt Eigen Matrix options, can usually be inferred.
/// \param pt Vertex of the point.
/// \param tv0, tv1, tv2 Vertices of the triangle.
/// \param [out] coordinates The Barycentric coordinates.
/// \return bool true on success, false if two or more if the triangle is considered degenerate
template <class T, int MOpt> inline
bool barycentricCoordinates(
	const Eigen::Matrix<T, 3, 1, MOpt>& pt,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
	Eigen::Matrix<T, 3, 1, MOpt>* coordinates)
{
	const Eigen::Matrix<T, 3, 1, MOpt> tn = (tv1 - tv0).cross(tv2 - tv0);
	return barycentricCoordinates(pt, tv0, tv1, tv2, tn, coordinates);
}

/// Check if a point is inside a triangle
/// \note Use barycentricCoordinates() if you need the coordinates
/// \pre The normal must be unit length
/// \pre The triangle vertices must be in counter clockwise order in respect to the normal
/// \tparam T			Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt		Eigen Matrix options, can usually be inferred.
/// \param pt			Vertex of the point.
/// \param tv0, tv1, tv2 Vertices of the triangle, must be in CCW.
/// \param tn			Normal of the triangle (yes must be of norm 1 and a,b,c CCW).
/// \return	true		if pt lies inside the triangle tv0, tv1, tv2, false otherwise.
template <class T, int MOpt> inline
bool isPointInsideTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& pt,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
	const Eigen::Matrix<T, 3, 1, MOpt>& tn)
{
	Eigen::Matrix<T, 3, 1, MOpt> baryCoords;
	bool result = barycentricCoordinates(pt, tv0, tv1, tv2, tn, &baryCoords);
	return (result &&
			baryCoords[0] >= -Geometry::ScalarEpsilon &&
			baryCoords[1] >= -Geometry::ScalarEpsilon &&
			baryCoords[2] >= -Geometry::ScalarEpsilon);
}

/// Check if a point is inside a triangle.
/// \note Use barycentricCoordinates() if you need the coordinates.
/// Please note that the normal will be calculated each time you use this call, if you are doing more than one
/// test with the same triangle, precalculate the normal and pass it. Into the other version of this function
/// \tparam T			Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt		Eigen Matrix options, can usually be inferred.
/// \param pt			Vertex of the point.
/// \param tv0, tv1, tv2 Vertices of the triangle, must be in CCW.
/// \return true if pt lies inside the triangle tv0, tv1, tv2, false otherwise.
template <class T, int MOpt> inline
bool isPointInsideTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& pt,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2)
{
	Eigen::Matrix<T, 3, 1, MOpt> baryCoords;
	bool result = barycentricCoordinates(pt, tv0, tv1, tv2, &baryCoords);
	return (result && baryCoords[0] >= -Geometry::ScalarEpsilon &&
			baryCoords[1] >= -Geometry::ScalarEpsilon &&
			baryCoords[2] >= -Geometry::ScalarEpsilon);
}

/// Check whether the points are coplanar.
/// \tparam T			Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt		Eigen Matrix options, can usually be inferred.
/// \param a, b, c, d	Points to check for coplanarity.
/// \return true if the points are coplanar.
template <class T, int MOpt> inline
bool isCoplanar(
	const Eigen::Matrix<T, 3, 1, MOpt>& a,
	const Eigen::Matrix<T, 3, 1, MOpt>& b,
	const Eigen::Matrix<T, 3, 1, MOpt>& c,
	const Eigen::Matrix<T, 3, 1, MOpt>& d)
{
	return std::abs((c - a).dot((b - a).cross(d - c))) < Geometry::ScalarEpsilon;
}

/// Calculate the normal distance between a point and a line.
/// \param pt		The input point.
/// \param v0,v1	Two vertices on the line.
/// \param [out] result The point projected onto the line.
/// \return			The normal distance between the point and the line
template <class T, int MOpt> inline
T distancePointLine(
	const Eigen::Matrix<T, 3, 1, MOpt>& pt,
	const Eigen::Matrix<T, 3, 1, MOpt>& v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& v1,
	Eigen::Matrix<T, 3, 1, MOpt>* result)
{
	// The lines is parametrized by:
	//		q = v0 + lambda0 * (v1-v0)
	// and we solve for pq.v01 = 0;
	Eigen::Matrix<T, 3, 1, MOpt> v01 = v1 - v0;
	T v01_norm2 = v01.squaredNorm();
	if (v01_norm2 <= Geometry::SquaredDistanceEpsilon)
	{
		*result = v0; // closest point is either
		T pv_norm2 = (pt - v0).squaredNorm();
		return sqrt(pv_norm2);
	}
	T lambda = (v01).dot(pt - v0);
	*result = v0 + lambda * v01 / v01_norm2;
	return (*result - pt).norm();
}

/// Point segment distance, if the projection of the closest point is not within the segments, the
/// closest segment point is used for the distance calculation.
/// \tparam T			Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt		Eigen Matrix options, can usually be inferred.
/// \param	pt		  	The input point
/// \param	sv0,sv1	  	The segment extremities.
/// \param [out] result	Either the projection onto the segment or one of the 2 vertices.
/// \return				The distance of the point from the segment.
template <class T, int MOpt> inline
T distancePointSegment(
	const Eigen::Matrix<T, 3, 1, MOpt>& pt,
	const Eigen::Matrix<T, 3, 1, MOpt>& sv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& sv1,
	Eigen::Matrix<T, 3, 1, MOpt>* result)
{
	Eigen::Matrix<T, 3, 1, MOpt> v01 = sv1 - sv0;
	T v01Norm2 = v01.squaredNorm();
	if (v01Norm2 <= Geometry::SquaredDistanceEpsilon)
	{
		*result = sv0; // closest point is either
		return (pt - sv0).norm();
	}
	T lambda = v01.dot(pt - sv0);
	if (lambda <= 0)
	{
		*result = sv0;
	}
	else if (lambda >= v01Norm2)
	{
		*result = sv1;
	}
	else
	{
		*result = sv0 + lambda * v01 / v01Norm2;
	}
	return (*result - pt).norm();
}

/// Determine the distance between two lines
/// \tparam T			Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt		Eigen Matrix options, can usually be inferred.
/// \param l0v0, l0v1	Points on Line 0.
/// \param l1v0, l1v1	Points on Line 1.
/// \param [out] pt0	The closest point on line 0.
/// \param [out] pt1	The closest point on line 1.
/// \return				The normal distance between the two given lines i.e. (pt0 - pt1).norm()
/// \note We are using distancePointSegment for the degenerate cases as opposed to
/// 	  distancePointLine, why is that ??? (HS-2013-apr-26)
template <class T, int MOpt> inline
T distanceLineLine(
	const Eigen::Matrix<T, 3, 1, MOpt>& l0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& l0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& l1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& l1v1,
	Eigen::Matrix<T, 3, 1, MOpt>* pt0,
	Eigen::Matrix<T, 3, 1, MOpt>* pt1)
{
	// Based on the outline of http://www.geometrictools.com/Distance.html, also refer to
	// http://geomalgorithms.com/a07-_distance.html for a geometric interpretation
	// The lines are parametrized by:
	//		p0 = l0v0 + lambda0 * (l0v1-l0v0)
	//		p1 = l1v0 + lambda1 * (l1v1-l1v0)
	// and we solve for p0p1 perpendicular to both lines
	T lambda0, lambda1;
	Eigen::Matrix<T, 3, 1, MOpt> l0v01 = l0v1 - l0v0;
	T a = l0v01.squaredNorm();
	if (a <= Geometry::SquaredDistanceEpsilon)
	{
		// Degenerate line 0
		*pt0 = l0v0;
		return distancePointSegment(l0v0, l1v0, l1v1, pt1);
	}
	Eigen::Matrix<T, 3, 1, MOpt> l1v01 = l1v1 - l1v0;
	T b = -l0v01.dot(l1v01);
	T c = l1v01.squaredNorm();
	if (c <= Geometry::SquaredDistanceEpsilon)
	{
		// Degenerate line 1
		*pt1 = l1v0;
		return distancePointSegment(l1v0, l0v0, l0v1, pt0);
	}
	Eigen::Matrix<T, 3, 1, MOpt> l0v0_l1v0 = l0v0 - l1v0;
	T d = l0v01.dot(l0v0_l1v0);
	T e = -l1v01.dot(l0v0_l1v0);
	T ratio = a * c - b * b;
	if (std::abs(ratio) <= Geometry::ScalarEpsilon)
	{
		// parallel case
		lambda0 = 0;
		lambda1 = e / c;
	}
	else
	{
		// non-parallel case
		T inv_ratio = T(1) / ratio;
		lambda0 = (b * e - c * d) * inv_ratio;
		lambda1 = (b * d - a * e) * inv_ratio;
	}
	*pt0 = l0v0 + lambda0 * l0v01;
	*pt1 = l1v0 + lambda1 * l1v01;
	return ((*pt0) - (*pt1)).norm();
}


/// Distance between two segments, if the project of the closest point is not on the opposing segment,
/// the segment endpoints will be used for the distance calculation
/// \tparam T			Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt		Eigen Matrix options, can usually be inferred.
/// \param s0v0, s0v1	Segment 0 Extremities.
/// \param s1v0, s1v1	Segment 1 Extremities.
/// \param [out] pt0	Closest point on segment 0
/// \param [out] pt1	Closest point on segment 1
/// \param [out] s0t	Abscissa at the point of intersection on Segment 0 (s0v0 + t * (s0v1 - s0v0)).
/// \param [out] s1t	Abscissa at the point of intersection on Segment 0 (s1v0 + t * (s1v1 - s1v0)).
/// \return Distance between the segments, i.e. (pt0 - pt1).norm()
template <class T, int MOpt>
T distanceSegmentSegment(
	const Eigen::Matrix<T, 3, 1, MOpt>& s0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& s0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& s1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& s1v1,
	Eigen::Matrix<T, 3, 1, MOpt>* pt0,
	Eigen::Matrix<T, 3, 1, MOpt>* pt1,
	T* s0t = nullptr,
	T* s1t = nullptr)
{
	// Based on the outline of http://www.geometrictools.com/Documentation/DistanceLine3Line3.pdf, also refer to
	// http://geomalgorithms.com/a07-_distance.html for a geometric interpretation
	// The segments are parametrized by:
	//		p0 = l0v0 + s * (l0v1-l0v0), with s between 0 and 1
	//		p1 = l1v0 + t * (l1v1-l1v0), with t between 0 and 1
	// We are minimizing Q(s, t) = as*as + 2bst + ct*ct + 2ds + 2et + f,
	Eigen::Matrix<T, 3, 1, MOpt> s0v01 = s0v1 - s0v0;
	T a = s0v01.squaredNorm();
	if (a <= Geometry::SquaredDistanceEpsilon)
	{
		// Degenerate segment 0
		*pt0 = s0v0;
		return distancePointSegment<T>(s0v0, s1v0, s1v1, pt1);
	}
	Eigen::Matrix<T, 3, 1, MOpt> s1v01 = s1v1 - s1v0;
	T b = -s0v01.dot(s1v01);
	T c = s1v01.squaredNorm();
	if (c <= Geometry::SquaredDistanceEpsilon)
	{
		// Degenerate segment 1
		*pt1 = s1v1;
		return distancePointSegment<T>(s1v0, s0v0, s0v1, pt0);
	}
	Eigen::Matrix<T, 3, 1, MOpt> tempLine = s0v0 - s1v0;
	T d = s0v01.dot(tempLine);
	T e = -s1v01.dot(tempLine);
	T ratio = a * c - b * b;
	T s, t; // parametrization variables (do not initialize)
	int region = -1;
	T tmp;
	// Non-parallel case
	if (1.0 - std::abs(s0v01.normalized().dot(s1v01.normalized())) >= Geometry::SquaredDistanceEpsilon)
	{
		// Get the region of the global minimum in the s-t space based on the line-line solution
		//		s=0		s=1
		//		^
		//		|		|
		//	4	|	3	|	2
		//	----|-------|-------	t=1
		//		|		|
		//	5	|	0	|	1
		//		|		|
		//	----|-------|------->	t=0
		//		|		|
		//	6	|	7	|	8
		//		|		|
		//
		s = b * e - c * d;
		t = b * d - a * e;
		if (s >= 0)
		{
			if (s <= ratio)
			{
				if (t >= 0)
				{
					if (t <= ratio)
					{
						region = 0;
					}
					else
					{
						region = 3;
					}
				}
				else
				{
					region = 7;
				}
			}
			else
			{
				if (t >= 0)
				{
					if (t <= ratio)
					{
						region = 1;
					}
					else
					{
						region = 2;
					}
				}
				else
				{
					region = 8;
				}
			}
		}
		else
		{
			if (t >= 0)
			{
				if (t <= ratio)
				{
					region = 5;
				}
				else
				{
					region = 4;
				}
			}
			else
			{
				region = 6;
			}
		}
		enum edge_type { s0, s1, t0, t1, edge_skip, edge_invalid };
		edge_type edge = edge_invalid;
		switch (region)
		{
			case 0:
				// Global minimum inside [0,1] [0,1]
				s /= ratio;
				t /= ratio;
				edge = edge_skip;
				break;
			case 1:
				edge = s1;
				break;
			case 2:
				// Q_s(1,1)/2 = a+b+d
				if (a + b + d > 0)
				{
					edge = t1;
				}
				else
				{
					edge = s1;
				}
				break;
			case 3:
				edge = t1;
				break;
			case 4:
				// Q_s(0,1)/2 = b+d
				if (b + d > 0)
				{
					edge = s0;
				}
				else
				{
					edge = t1;
				}
				break;
			case 5:
				edge = s0;
				break;
			case 6:
				// Q_s(0,0)/2 = d
				if (d > 0)
				{
					edge = s0;
				}
				else
				{
					edge = t0;
				}
				break;
			case 7:
				edge = t0;
				break;
			case 8:
				// Q_s(1,0)/2 = a+d
				if (a + d > 0)
				{
					edge = t0;
				}
				else
				{
					edge = s1;
				}
				break;
			default:
				break;
		}
		switch (edge)
		{
			case s0:
				// F(t) = Q(0,t), F?(t) = 2*(e+c*t)
				// F?(T) = 0 when T = -e/c, then clamp between 0 and 1 (c always >= 0)
				s = 0;
				tmp = e;
				if (tmp > 0)
				{
					t = 0;
				}
				else if (-tmp > c)
				{
					t = 1;
				}
				else
				{
					t = -tmp / c;
				}
				break;
			case s1:
				// F(t) = Q(1,t), F?(t) = 2*((b+e)+c*t)
				// F?(T) = 0 when T = -(b+e)/c, then clamp between 0 and 1 (c always >= 0)
				s = 1;
				tmp = b + e;
				if (tmp > 0)
				{
					t = 0;
				}
				else if (-tmp > c)
				{
					t = 1;
				}
				else
				{
					t = -tmp / c;
				}
				break;
			case t0:
				// F(s) = Q(s,0), F?(s) = 2*(d+a*s) =>
				// F?(S) = 0 when S = -d/a, then clamp between 0 and 1 (a always >= 0)
				t = 0;
				tmp = d;
				if (tmp > 0)
				{
					s = 0;
				}
				else if (-tmp > a)
				{
					s = 1;
				}
				else
				{
					s = -tmp / a;
				}
				break;
			case t1:
				// F(s) = Q(s,1), F?(s) = 2*(b+d+a*s) =>
				// F?(S) = 0 when S = -(b+d)/a, then clamp between 0 and 1  (a always >= 0)
				t = 1;
				tmp = b + d;
				if (tmp > 0)
				{
					s = 0;
				}
				else if (-tmp > a)
				{
					s = 1;
				}
				else
				{
					s = -tmp / a;
				}
				break;
			case edge_skip:
				break;
			default:
				break;
		}
	}
	else
		// Parallel case
	{
		if (b > 0)
		{
			// Segments have different directions
			if (d >= 0)
			{
				// 0-0 end points since s-segment 0 less than t-segment 0
				s = 0;
				t = 0;
			}
			else if (-d <= a)
			{
				// s-segment 0 end-point in the middle of the t 0-1 segment, get distance
				s = -d / a;
				t = 0;
			}
			else
			{
				// s-segment 1 is definitely closer
				s = 1;
				tmp = a + d;
				if (-tmp >= b)
				{
					t = 1;
				}
				else
				{
					t = -tmp / b;
				}
			}
		}
		else
		{
			// Both segments have the same dir
			if (-d >= a)
			{
				// 1-0
				s = 1;
				t = 0;
			}
			else if (d <= 0)
			{
				// mid-0
				s = -d / a;
				t = 0;
			}
			else
			{
				s = 0;
				// 1-mid
				if (d >= -b)
				{
					t = 1;
				}
				else
				{
					t = -d / b;
				}
			}
		}
	}
	*pt0 = s0v0 + s * (s0v01);
	*pt1 = s1v0 + t * (s1v01);
	if (s0t != nullptr && s1t != nullptr)
	{
		*s0t = s;
		*s1t = t;
	}
	return ((*pt1) - (*pt0)).norm();
}

/// Calculate the normal distance of a point from a triangle, the resulting point will be on the edge of the triangle
/// if the projection of the point is not inside the triangle.
/// \tparam T			Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt		Eigen Matrix options, can usually be inferred.
/// \param pt			The point that is being measured.
/// \param tv0, tv1, tv2 The vertices of the triangle.
/// \param [out] result	The point on the triangle that is closest to pt, if the projection of pt onto the triangle.
/// 					plane is not inside the triangle the closest point on the edge will be used.
/// \return				The distance between the point and the triangle, i.e (result - pt).norm()
template <class T, int MOpt> inline
T distancePointTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& pt,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
	Eigen::Matrix<T, 3, 1, MOpt>* result)
{
	// Based on the outline of http://www.geometrictools.com/Distance.html, also refer to
	//	http://softsurfer.com/Archive/algorithm_0106 for a geometric interpretation
	// The triangle is parametrized by:
	//		t: tv0 + s * (tv1-tv0) + t * (tv2-tv0) , with s and t between 0 and 1
	// We are minimizing Q(s, t) = as*as + 2bst + ct*ct + 2ds + 2et + f,
	Eigen::Matrix<T, 3, 1, MOpt> tv01 = tv1 - tv0;
	Eigen::Matrix<T, 3, 1, MOpt> tv02 = tv2 - tv0;
	T a = tv01.squaredNorm();
	if (a <= Geometry::SquaredDistanceEpsilon)
	{
		// Degenerate edge 1
		return distancePointSegment<T>(pt, tv0, tv2, result);
	}
	T b = tv01.dot(tv02);
	T tCross = tv01.cross(tv02).squaredNorm();
	if (tCross <= Geometry::SquaredDistanceEpsilon)
	{
		// Degenerate edge 2
		return distancePointSegment<T>(pt, tv0, tv1, result);
	}
	T c = tv02.squaredNorm();
	if (c <= Geometry::SquaredDistanceEpsilon)
	{
		// Degenerate edge 3
		return distancePointSegment<T>(pt, tv0, tv1, result);
	}
	Eigen::Matrix<T, 3, 1, MOpt> tv0pv0 = tv0 - pt;
	T d = tv01.dot(tv0pv0);
	T e = tv02.dot(tv0pv0);
	T ratio = a * c - b * b;
	T s = b * e - c * d;
	T t = b * d - a * e;
	// Determine region (inside-outside triangle)
	int region = -1;
	if (s + t <= ratio)
	{
		if (s < 0)
		{
			if (t < 0)
			{
				region = 4;
			}
			else
			{
				region = 3;
			}
		}
		else if (t < 0)
		{
			region = 5;
		}
		else
		{
			region = 0;
		}
	}
	else
	{
		if (s < 0)
		{
			region = 2;
		}
		else if (t < 0)
		{
			region = 6;
		}
		else
		{
			region = 1;
		}
	}
	//	Regions:                    /
	//	    ^ t=0                   /
	//	 \ 2|                       /
	//	  \ |                       /
	//	   \|                       /
	//		\                       /
	//		|\                      /
	//		| \	  1                 /
	//	3	|  \                    /
	//		| 0 \                   /
	//	----|----\------->	s=0     /
	//		| 	  \                 /
	//	4	|	5  \   6            /
	//		|	    \               /
	//                              /
	T numer, denom, tmp0, tmp1;
	enum edge_type { s0, t0, s1t1, edge_skip, edge_invalid };
	edge_type edge = edge_invalid;
	switch (region)
	{
		case 0:
			// Global minimum inside [0,1] [0,1]
			numer = T(1) / ratio;
			s *= numer;
			t *= numer;
			edge = edge_skip;
			break;
		case 1:
			edge = s1t1;
			break;
		case 2:
			// Grad(Q(0,1)).(0,-1)/2 = -c-e
			// Grad(Q(0,1)).(1,-1)/2 = b=d-c-e
			tmp0 = b + d;
			tmp1 = c + e;
			if (tmp1 > tmp0)
			{
				edge = s1t1;
			}
			else
			{
				edge = s0;
			}
			break;
		case 3:
			edge = s0;
			break;
		case 4:
			// Grad(Q(0,0)).(0,1)/2 = e
			// Grad(Q(0,0)).(1,0)/2 = d
			if (e >= d)
			{
				edge = t0;
			}
			else
			{
				edge = s0;
			}
			break;
		case 5:
			edge = t0;
			break;
		case 6:
			// Grad(Q(1,0)).(-1,0)/2 = -a-d
			// Grad(Q(1,0)).(-1,1)/2 = -a-d+b+e
			tmp0 = -a - d;
			tmp1 = -a - d + b + e;
			if (tmp1 > tmp0)
			{
				edge = t0;
			}
			else
			{
				edge = s1t1;
			}
			break;
		default:
			break;
	}
	switch (edge)
	{
		case s0:
			// F(t) = Q(0, t), F'(t)=0 when -e/c = 0
			s = 0;
			if (e >= 0)
			{
				t = 0;
			}
			else
			{
				t = (-e >= c ? 1 : -e / c);
			}
			break;
		case t0:
			// F(s) = Q(s, 0), F'(s)=0 when -d/a = 0
			t = 0;
			if (d >= 0)
			{
				s = 0;
			}
			else
			{
				s = (-d >= a ? 1 : -d / a);
			}
			break;
		case s1t1:
			// F(s) = Q(s, 1-s), F'(s) = 0 when (c+e-b-d)/(a-2b+c) = 0 (denom = || tv01-tv02 ||^2 always > 0)
			numer = c + e - b - d;
			if (numer <= 0)
			{
				s = 0;
			}
			else
			{
				denom = a - 2 * b + c;
				s = (numer >= denom ? 1 : numer / denom);
			}
			t = 1 - s;
			break;
		case edge_skip:
			break;
		default:
			break;
	}
	*result = tv0 + s * tv01 + t * tv02;
	return ((*result) - pt).norm();
}

/// Calculate the intersection of a line segment with a triangle
/// See http://geomalgorithms.com/a06-_intersect-2.html#intersect_RayTriangle for the algorithm
/// \pre The normal must be unit length
/// \pre The triangle vertices must be in counter clockwise order in respect to the normal
/// \tparam T			Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt		Eigen Matrix options, can usually be inferred.
/// \param sv0,sv1		Extremities of the segment.
/// \param tv0,tv1,tv2	The triangle vertices. CCW around the normal.
/// \param tn			The triangle normal, should be normalized.
/// \param [out] result	The point where the triangle and the line segment intersect, invalid if they don't intersect.
/// \return true if the segment intersects with the triangle, false if it does not
/// \note HS-2013-may-07 This is the only function that only checks for intersection rather than returning a distance
/// 	  if necessary this should be rewritten to do the distance calculation, doing so would necessitate to check
/// 	  against all the triangle edges in the non intersection cases.
template <class T, int MOpt> inline
bool doesCollideSegmentTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& sv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& sv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
	const Eigen::Matrix<T, 3, 1, MOpt>& tn,
	Eigen::Matrix<T, 3, 1, MOpt>* result)
{
	// Triangle edges vectors
	Eigen::Matrix<T, 3, 1, MOpt> u = tv1 - tv0;
	Eigen::Matrix<T, 3, 1, MOpt> v = tv2 - tv0;

	// Ray direction vector
	Eigen::Matrix<T, 3, 1, MOpt> dir = sv1 - sv0;
	Eigen::Matrix<T, 3, 1, MOpt> w0 = sv0 - tv0;
	T a = -tn.dot(w0);
	T b = tn.dot(dir);

	result->setConstant((std::numeric_limits<double>::quiet_NaN()));

	// Ray is parallel to triangle plane
	if (std::abs(b) <= Geometry::AngularEpsilon)
	{
		if (std::abs(a) <= Geometry::AngularEpsilon)
		{
			// Ray lies in triangle plane
			Eigen::Matrix<T, 3, 1, MOpt> baryCoords;
			for (int i = 0; i < 2; ++i)
			{
				barycentricCoordinates((i == 0 ? sv0 : sv1), tv0, tv1, tv2, tn, &baryCoords);
				if (baryCoords[0] >= 0 && baryCoords[1] >= 0 && baryCoords[2] >= 0)
				{
					*result = (i == 0) ? sv0 : sv1;
					return true;
				}
			}
			// All segment endpoints outside of triangle
			return false;
		}
		else
		{
			// Segment parallel to triangle but not in same plane
			return false;
		}
	}

	// Get intersect point of ray with triangle plane
	T r = a / b;
	// Ray goes away from triangle
	if (r < -Geometry::DistanceEpsilon)
	{
		return false;
	}
	//Ray comes toward triangle but isn't long enough to reach it
	if (r > 1 + Geometry::DistanceEpsilon)
	{
		return false;
	}

	// Intersect point of ray and plane
	Eigen::Matrix<T, 3, 1, MOpt> presumedIntersection = sv0 + r * dir;
	// Collision point inside T?
	T uu = u.dot(u);
	T uv = u.dot(v);
	T vv = v.dot(v);
	Eigen::Matrix<T, 3, 1, MOpt> w = presumedIntersection - tv0;
	T wu = w.dot(u);
	T wv = w.dot(v);
	T D = uv * uv - uu * vv;
	// Get and test parametric coords
	T s = (uv * wv - vv * wu) / D;
	// I is outside T
	if (s < -Geometry::DistanceEpsilon || s > 1 + Geometry::DistanceEpsilon)
	{
		return false;
	}
	T t = (uv * wu - uu * wv) / D;
	// I is outside T
	if (t < -Geometry::DistanceEpsilon || (s + t) > 1 + Geometry::DistanceEpsilon)
	{
		return false;
	}
	// I is in T
	*result = sv0 + r * dir;
	return true;
}


/// Calculate the distance of a point to a plane
/// \pre n needs to the normalized
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param pt		The point to check.
/// \param n		The normal of the plane n (normalized).
/// \param d		Constant d for the plane equation as in n.x + d = 0.
/// \param [out] result Projection of point p into the plane.
/// \return			The distance to the plane (negative if on the backside of the plane).
template <class T, int MOpt> inline
T distancePointPlane(
	const Eigen::Matrix<T, 3, 1, MOpt>& pt,
	const Eigen::Matrix<T, 3, 1, MOpt>& n,
	T d,
	Eigen::Matrix<T, 3, 1, MOpt>* result)
{
	T dist = n.dot(pt) + d;
	*result = pt - n * dist;
	return dist;
}


/// Calculate the distance between a segment and a plane.
/// \pre n should be normalized
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param sv0,sv1	Endpoints of the segments.
/// \param n		Normal of the plane n (normalized).
/// \param d		Constant d in n.x + d = 0.
/// \param [out] closestPointSegment Point closest to the plane, the midpoint of the segment (v0+v1)/2
/// 				is being used if the segment is parallel to the plane. If the segment actually
/// 				intersects the plane segmentIntersectionPoint will be equal to planeIntersectionPoint.
/// \param [out] planeIntersectionPoint the point on the plane where the line defined by the segment
/// 				intersects the plane.
/// \return			the distance of closest point of the segment to the plane, 0 if the segment intersects the plane,
/// 				negative if the closest point is on the other side of the plane.
template <class T, int MOpt> inline
T distanceSegmentPlane(
	const Eigen::Matrix<T, 3, 1, MOpt>& sv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& sv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& n,
	T d,
	Eigen::Matrix<T, 3, 1, MOpt>* closestPointSegment,
	Eigen::Matrix<T, 3, 1, MOpt>* planeIntersectionPoint)
{
	T dist0 = n.dot(sv0) + d;
	T dist1 = n.dot(sv1) + d;
	// Parallel case
	Eigen::Matrix<T, 3, 1, MOpt> v01 = sv1 - sv0;
	if (std::abs(n.dot(v01)) <= Geometry::AngularEpsilon)
	{
		*closestPointSegment = (sv0 + sv1) * T(0.5);
		dist0 = n.dot(*closestPointSegment) + d;
		*planeIntersectionPoint = *closestPointSegment - dist0 * n;
		return (std::abs(dist0) < Geometry::DistanceEpsilon ? 0 : dist0);
	}
	// Both on the same side
	if ((dist0 > 0 && dist1 > 0) || (dist0 < 0 && dist1 < 0))
	{
		if (std::abs(dist0) < std::abs(dist1))
		{
			*closestPointSegment = sv0;
			*planeIntersectionPoint = sv0 - dist0 * n;
			return dist0;
		}
		else
		{
			*closestPointSegment = sv1;
			*planeIntersectionPoint = sv1 - dist1 * n;
			return dist1;
		}
	}
	// Segment cutting through
	else
	{
		Eigen::Matrix<T, 3, 1, MOpt> v01 = sv1 - sv0;
		T lambda = (-d - sv0.dot(n)) / v01.dot(n);
		*planeIntersectionPoint = sv0 + lambda * v01;
		*closestPointSegment = *planeIntersectionPoint;
		return 0;
	}
}


/// Calculate the distance of a triangle to a plane.
/// \pre n should be normalized.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param tv0,tv1,tv2 Points of the triangle.
/// \param n		Normal of the plane n (normalized).
/// \param d		Constant d in n.x + d = 0.
/// \param closestPointTriangle Closest point on the triangle, when the triangle is coplanar to
/// 				the plane (tv0+tv1+tv2)/3 is used, when the triangle intersects the plane the midpoint of
/// 				the intersection segment is returned.
/// \param planeProjectionPoint Projection of the closest point onto the plane, when the triangle intersects
/// 				the plane the midpoint of the intersection segment is returned.
/// \return The distance of the triangle to the plane.
template <class T, int MOpt> inline
T distanceTrianglePlane(
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
	const Eigen::Matrix<T, 3, 1, MOpt>& n,
	T d,
	Eigen::Matrix<T, 3, 1, MOpt>* closestPointTriangle,
	Eigen::Matrix<T, 3, 1, MOpt>* planeProjectionPoint)
{
	Eigen::Matrix<T, 3, 1, MOpt> distances(n.dot(tv0) + d, n.dot(tv1) + d, n.dot(tv2) + d);
	Eigen::Matrix<T, 3, 1, MOpt> t01 = tv1 - tv0;
	Eigen::Matrix<T, 3, 1, MOpt> t02 = tv2 - tv0;
	Eigen::Matrix<T, 3, 1, MOpt> t12 = tv2 - tv1;

	closestPointTriangle->setConstant((std::numeric_limits<double>::quiet_NaN()));
	planeProjectionPoint->setConstant((std::numeric_limits<double>::quiet_NaN()));

	// HS-2013-may-09 Could there be a case where we fall into the wrong tree because of the checks against
	// the various epsilon values all going against us ???
	// Parallel case (including Coplanar)
	if (std::abs(n.dot(t01)) <= Geometry::AngularEpsilon && std::abs(n.dot(t02)) <= Geometry::AngularEpsilon)
	{
		*closestPointTriangle = (tv0 + tv1 + tv2) / T(3);
		*planeProjectionPoint = *closestPointTriangle - n * distances[0];
		return distances[0];
	}

	// Is there an intersection
	if ((distances.array() < -Geometry::DistanceEpsilon).any() &&
		(distances.array() > Geometry::DistanceEpsilon).any())
	{
		if (distances[0] * distances[1] < 0)
		{
			*closestPointTriangle = tv0 + (-d - n.dot(tv0)) / n.dot(t01) * t01;
			if (distances[0] * distances[2] < 0)
			{
				*planeProjectionPoint = tv0 + (-d - n.dot(tv0)) / n.dot(t02) * t02;
			}
			else
			{
				Eigen::Matrix<T, 3, 1, MOpt> t12 = tv2 - tv1;
				*planeProjectionPoint = tv1 + (-d - n.dot(tv1)) / n.dot(t12) * t12;
			}
		}
		else
		{
			*closestPointTriangle = tv0 + (-d - n.dot(tv0)) / n.dot(t02) * t02;
			*planeProjectionPoint = tv1 + (-d - n.dot(tv1)) / n.dot(t12) * t12;
		}

		// Find the midpoint, take this out to return the segment endpoints
		*closestPointTriangle = *planeProjectionPoint = (*closestPointTriangle + *planeProjectionPoint) * T(0.5);
		return 0;
	}

	int index;
	distances.cwiseAbs().minCoeff(&index);
	switch (index)
	{
		case 0: //distances[0] is closest
			*closestPointTriangle = tv0;
			*planeProjectionPoint = tv0 - n * distances[0];
			return distances[0];
		case 1: //distances[1] is closest
			*closestPointTriangle = tv1;
			*planeProjectionPoint = tv1 - n * distances[1];
			return distances[1];
		case 2: //distances[2] is closest
			*closestPointTriangle = tv2;
			*planeProjectionPoint = tv2 - n * distances[2];
			return distances[2];
	}

	return std::numeric_limits<T>::quiet_NaN();
}

/// Test if two planes are intersecting, if yes also calculate the intersection line.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param pn0,pd0	Normal and constant of the first plane, nx + d = 0.
/// \param pn1,pd1	Normal and constant of the second plane, nx + d = 0.
/// \param [out] pt0,pt1 Two points on the intersection line, not valid if there is no intersection.
/// \return true when a unique line exists, false for disjoint or coinciding.
template <class T, int MOpt> inline
bool doesIntersectPlanePlane(
	const Eigen::Matrix<T, 3, 1, MOpt>& pn0, T pd0,
	const Eigen::Matrix<T, 3, 1, MOpt>& pn1, T pd1,
	Eigen::Matrix<T, 3, 1, MOpt>* pt0,
	Eigen::Matrix<T, 3, 1, MOpt>* pt1)
{
	// Algorithm from real time collision detection - optimized version page 210 (with extra checks)
	const Eigen::Matrix<T, 3, 1, MOpt> lineDir = pn0.cross(pn1);
	const T lineDirNorm2 = lineDir.squaredNorm();

	pt0->setConstant((std::numeric_limits<double>::quiet_NaN()));
	pt1->setConstant((std::numeric_limits<double>::quiet_NaN()));

	// Test if the two planes are parallel
	if (lineDirNorm2 <= Geometry::SquaredDistanceEpsilon)
	{
		return false; // planes disjoint
	}
	// Compute common point
	*pt0 = (pd1 * pn0 - pd0 * pn1).cross(lineDir) / lineDirNorm2;
	*pt1 = *pt0 + lineDir;
	return true;
}


/// Calculate the distance of a line segment to a triangle.
/// Note that this version will calculate the normal of the triangle,
/// if the normal is known use the other version of this function.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param sv0,sv1	Extremities of the line segment.
/// \param tv0, tv1, tv2 Triangle points.
/// \param [out] segmentPoint Closest point on the segment.
/// \param [out] trianglePoint Closest point on the triangle.
/// \return the the distance between the two closest points, i.e. (trianglePoint - segmentPoint).norm().
template <class T, int MOpt> inline
T distanceSegmentTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& sv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& sv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
	Eigen::Matrix<T, 3, 1, MOpt>* segmentPoint,
	Eigen::Matrix<T, 3, 1, MOpt>* trianglePoint)
{
	Eigen::Matrix<T, 3, 1, MOpt> n = (tv1 - tv0).cross(tv2 - tv1);
	n.normalize();
	return distanceSegmentTriangle(sv0, sv1, tv0, tv1, tv2, n, segmentPoint, trianglePoint);
}

/// Calculate the distance of a line segment to a triangle.
/// \pre n needs to be normalized.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param sv0,sv1	Extremities of the line segment.
/// \param tv0, tv1, tv2 Points of the triangle.
/// \param normal		Normal of the triangle (Expected to be normalized)
/// \param [out] segmentPoint Closest point on the segment.
/// \param [out] trianglePoint Closest point on the triangle.
/// \return the distance between the two closest points.
template <class T, int MOpt> inline
T distanceSegmentTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& sv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& sv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
	const Eigen::Matrix<T, 3, 1, MOpt>& normal,
	Eigen::Matrix<T, 3, 1, MOpt>* segmentPoint,
	Eigen::Matrix<T, 3, 1, MOpt>* trianglePoint)
{
	segmentPoint->setConstant((std::numeric_limits<double>::quiet_NaN()));
	trianglePoint->setConstant((std::numeric_limits<double>::quiet_NaN()));

	// Setting up the plane that the triangle is in
	const Eigen::Matrix<T, 3, 1, MOpt>& n = normal;
	T d = -n.dot(tv0);
	Eigen::Matrix<T, 3, 1, MOpt> baryCoords;
	// Degenerate case: Line and triangle plane parallel
	const Eigen::Matrix<T, 3, 1, MOpt> v01 = sv1 - sv0;
	const T v01DotTn = n.dot(v01);
	if (std::abs(v01DotTn) <= Geometry::AngularEpsilon)
	{
		// Check if any of the points project onto the tri
		// otherwise normal (non-parallel) processing will get the right result
		T dst = std::abs(distancePointPlane(sv0, n, d, trianglePoint));
		Eigen::Matrix<T, 3, 1, MOpt> baryCoords;
		barycentricCoordinates(*trianglePoint, tv0, tv1, tv2, normal, &baryCoords);
		if (baryCoords[0] >= 0 && baryCoords[1] >= 0 && baryCoords[2] >= 0)
		{
			*segmentPoint = sv0;
			return dst;
		}
		dst = std::abs(distancePointPlane(sv1, n, d, trianglePoint));
		barycentricCoordinates(*trianglePoint, tv0, tv1, tv2, normal, &baryCoords);
		if (baryCoords[0] >= 0 && baryCoords[1] >= 0 && baryCoords[2] >= 0)
		{
			*segmentPoint = sv1;
			return dst;
		}
	}
	// Line and triangle plane *not* parallel: check cut through case only, the rest will be check later
	else
	{
		T lambda = -n.dot(sv0 - tv0) / v01DotTn;
		if (lambda >= 0 && lambda <= 1)
		{
			*segmentPoint = *trianglePoint = sv0 + lambda * v01;
			barycentricCoordinates(*trianglePoint, tv0, tv1, tv2, normal, &baryCoords);
			if (baryCoords[0] >= 0 && baryCoords[1] >= 0 && baryCoords[2] >= 0)
			{
				// Segment goes through the triangle
				return 0;
			}
		}
	}
	// At this point the segment is nearest point to one of the triangle edges or one of the end points
	Eigen::Matrix<T, 3, 1, MOpt> segColPt01, segColPt02, segColPt12, triColPt01, triColPt02, triColPt12;
	T dst01 = distanceSegmentSegment(sv0, sv1, tv0, tv1, &segColPt01, &triColPt01);
	T dst02 = distanceSegmentSegment(sv0, sv1, tv0, tv2, &segColPt02, &triColPt02);
	T dst12 = distanceSegmentSegment(sv0, sv1, tv1, tv2, &segColPt12, &triColPt12);
	Eigen::Matrix<T, 3, 1, MOpt> ptTriCol0, ptTriCol1;
	T dstPtTri0 = std::abs(distancePointPlane(sv0, n, d, &ptTriCol0));
	barycentricCoordinates(ptTriCol0, tv0, tv1, tv2, normal, &baryCoords);
	if (baryCoords[0] < 0 || baryCoords[1] < 0 || baryCoords[2] < 0)
	{
		dstPtTri0 = std::numeric_limits<T>::max();
	}
	T dstPtTri1 = std::abs(distancePointPlane(sv1, n, d, &ptTriCol1));
	barycentricCoordinates(ptTriCol1, tv0, tv1, tv2, normal, &baryCoords);
	if (baryCoords[0] < 0 || baryCoords[1] < 0 || baryCoords[2] < 0)
	{
		dstPtTri1 = std::numeric_limits<T>::max();
	}

	int minIndex;
	Eigen::Matrix<double, 5, 1> distances;
	(distances << dst01, dst02, dst12, dstPtTri0, dstPtTri1).finished().minCoeff(&minIndex);
	switch (minIndex)
	{
		case 0:
			*segmentPoint = segColPt01;
			*trianglePoint = triColPt01;
			return dst01;
		case 1:
			*segmentPoint = segColPt02;
			*trianglePoint = triColPt02;
			return dst02;
		case 2:
			*segmentPoint = segColPt12;
			*trianglePoint = triColPt12;
			return dst12;
		case 3:
			*segmentPoint = sv0;
			*trianglePoint = ptTriCol0;
			return dstPtTri0;
		case 4:
			*segmentPoint = sv1;
			*trianglePoint = ptTriCol1;
			return dstPtTri1;
	}

	// Invalid ...
	return std::numeric_limits<T>::quiet_NaN();

}


/// Calculate the distance between two triangles
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param t0v0,t0v1,t0v2 Points of the first triangle.
/// \param t1v0,t1v1,t1v2 Points of the second triangle.
/// \param [out] closestPoint0 Closest point on the first triangle, unless penetrating,
/// 			 in which case it is the point along the edge that allows min separation.
/// \param [out] closestPoint1 Closest point on the second triangle, unless penetrating,
/// 			 in which case it is the point along the edge that allows min separation.
/// \return the distance between the two triangles.
template <class T, int MOpt> inline
T distanceTriangleTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	Eigen::Matrix<T, 3, 1, MOpt>* closestPoint0,
	Eigen::Matrix<T, 3, 1, MOpt>* closestPoint1)
{
	// Check the segments of t0 against t1
	T minDst = std::numeric_limits<T>::max();
	T currDst = 0;
	Eigen::Matrix<T, 3, 1, MOpt> segPt, triPt;
	Eigen::Matrix<T, 3, 1, MOpt> n0 = (t0v1 - t0v0).cross(t0v2 - t0v0);
	n0.normalize();
	Eigen::Matrix<T, 3, 1, MOpt> n1 = (t1v1 - t1v0).cross(t1v2 - t1v0);
	n1.normalize();
	currDst = distanceSegmentTriangle(t0v0, t0v1, t1v0, t1v1, t1v2, n1, &segPt, &triPt);
	if (currDst < minDst)
	{
		minDst = currDst;
		*closestPoint0 = segPt;
		*closestPoint1 = triPt;
	}
	currDst = distanceSegmentTriangle(t0v1, t0v2, t1v0, t1v1, t1v2, n1, &segPt, &triPt);
	if (currDst < minDst)
	{
		minDst = currDst;
		*closestPoint0 = segPt;
		*closestPoint1 = triPt;
	}
	currDst = distanceSegmentTriangle(t0v2, t0v0, t1v0, t1v1, t1v2, n1, &segPt, &triPt);
	if (currDst < minDst)
	{
		minDst = currDst;
		*closestPoint0 = segPt;
		*closestPoint1 = triPt;
	}
	// Check the segments of t1 against t0
	currDst = distanceSegmentTriangle(t1v0, t1v1, t0v0, t0v1, t0v2, n0, &segPt, &triPt);
	if (currDst < minDst)
	{
		minDst = currDst;
		*closestPoint1 = segPt;
		*closestPoint0 = triPt;
	}
	currDst = distanceSegmentTriangle(t1v1, t1v2, t0v0, t0v1, t0v2, n0, &segPt, &triPt);
	if (currDst < minDst)
	{
		minDst = currDst;
		*closestPoint1 = segPt;
		*closestPoint0 = triPt;
	}
	currDst = distanceSegmentTriangle(t1v2, t1v0, t0v0, t0v1, t0v2, n0, &segPt, &triPt);
	if (currDst < minDst)
	{
		minDst = currDst;
		*closestPoint1 = segPt;
		*closestPoint0 = triPt;
	}
	return (minDst);
}

/// Calculate the intersections between a line segment and an axis aligned box
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param sv0,sv1	Extremities of the line segment.
/// \param box		Axis aligned bounding box
/// \param [out] intersections The points of intersection between the segment and the box
template <class T, int MOpt>
void intersectionsSegmentBox(
	const Eigen::Matrix<T, 3, 1, MOpt>& sv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& sv1,
	const Eigen::AlignedBox<T, 3>& box,
	std::vector<Eigen::Matrix<T, 3, 1, MOpt>>* intersections)
{
	Eigen::Array<T, 3, 1, MOpt> v01 = sv1 - sv0;
	Eigen::Array<bool, 3, 1, MOpt> parallelToPlane = (v01.cwiseAbs().array() < Geometry::DistanceEpsilon);
	if (parallelToPlane.any())
	{
		Eigen::Array<bool, 3, 1, MOpt> beyondMinCorner = (sv0.array() < box.min().array());
		Eigen::Array<bool, 3, 1, MOpt> beyondMaxCorner = (sv0.array() > box.max().array());
		if ((parallelToPlane && (beyondMinCorner || beyondMaxCorner)).any())
		{
			return;
		}
	}

	// Calculate the intersection of the segment with each of the 6 box planes.
	// The intersection is calculated as the distance along the segment (abscissa)
	// scaled from 0 to 1.
	Eigen::Array<T, 3, 2, MOpt> planeIntersectionAbscissas;
	planeIntersectionAbscissas.col(0) = (box.min().array() - sv0.array());
	planeIntersectionAbscissas.col(1) = (box.max().array() - sv0.array());

	// While we could be dividing by zero here, INF values are
	// correctly handled by the rest of the function.
	planeIntersectionAbscissas.colwise() /= v01;

	T entranceAbscissa = planeIntersectionAbscissas.rowwise().minCoeff().maxCoeff();
	T exitAbscissa = planeIntersectionAbscissas.rowwise().maxCoeff().minCoeff();
	if (entranceAbscissa < exitAbscissa && exitAbscissa > T(0.0))
	{
		if (entranceAbscissa >= T(0.0) && entranceAbscissa <= T(1.0))
		{
			intersections->push_back(sv0 + v01.matrix() * entranceAbscissa);
		}

		if (exitAbscissa >= T(0.0) && exitAbscissa <= T(1.0))
		{
			intersections->push_back(sv0 + v01.matrix() * exitAbscissa);
		}
	}
}

/// Test if an axis aligned box intersects with a capsule
/// \tparam T Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt Eigen Matrix options, can usually be inferred.
/// \param capsuleBottom Position of the capsule bottom
/// \param capsuleTop Position of the capsule top
/// \param capsuleRadius The capsule radius
/// \param box Axis aligned bounding box
/// \return True, if intersection is detected.
template <class T, int MOpt>
bool doesIntersectBoxCapsule(
	const Eigen::Matrix<T, 3, 1, MOpt>& capsuleBottom,
	const Eigen::Matrix<T, 3, 1, MOpt>& capsuleTop,
	const T capsuleRadius,
	const Eigen::AlignedBox<T, 3>& box)
{
	Eigen::AlignedBox<double, 3> dilatedBox(box.min().array() - capsuleRadius, box.max().array() + capsuleRadius);
	std::vector<Vector3d> candidates;
	intersectionsSegmentBox(capsuleBottom, capsuleTop, dilatedBox, &candidates);
	if (dilatedBox.contains(capsuleBottom))
	{
		candidates.push_back(capsuleBottom);
	}
	if (dilatedBox.contains(capsuleTop))
	{
		candidates.push_back(capsuleTop);
	}

	bool doesIntersect = false;
	ptrdiff_t dimensionsOutsideBox;
	Vector3d clampedPosition, segmentPoint;
	for (auto candidate = candidates.cbegin(); candidate != candidates.cend(); ++candidate)
	{
		// Collisions between a capsule and a box are the same as a segment and a dilated
		// box with rounded corners. If the intersection occurs outside the original box
		// in two dimensions (collision with an edge of the dilated box) or three
		// dimensions (collision with the corner of the dilated box) dimensions, we need
		// to check if it is inside these rounded corners.
		dimensionsOutsideBox = (candidate->array() > box.max().array()).count();
		dimensionsOutsideBox += (candidate->array() < box.min().array()).count();
		if (dimensionsOutsideBox >= 2)
		{
			clampedPosition = (*candidate).array().min(box.max().array()).max(box.min().array());
			if (distancePointSegment(clampedPosition, capsuleBottom, capsuleTop, &segmentPoint) > capsuleRadius)
			{
				// Doesn't intersect, try the next candidate.
				continue;
			}
		}
		doesIntersect = true;
		break;
	}
	return doesIntersect;
}

/// Helper method to determine the nearest point between a point and a line.
/// \tparam T the numeric data type used for the vector argument. Can usually be deduced.
/// \tparam VOpt the option flags (alignment etc.) used for the vector argument. Can be deduced.
/// \param point is the point under consideration.
/// \param segment0 one point on the line
/// \param segment1 second point on the line
/// \return the closest point on the line through the segment to the point under test
template <class T, int VOpt>
Eigen::Matrix<T, 3, 1, VOpt> nearestPointOnLine(const Eigen::Matrix<T, 3, 1, VOpt>& point,
		const Eigen::Matrix<T, 3, 1, VOpt>& segment0, const Eigen::Matrix<T, 3, 1, VOpt>& segment1)
{
	auto pointToSegmentStart = segment0 - point;
	auto segmentDirection = segment1 - segment0;
	auto squaredNorm = segmentDirection.squaredNorm();
	SURGSIM_ASSERT(squaredNorm != 0.0) << "Line is defined by two collocated points.";
	auto distance = -pointToSegmentStart.dot(segmentDirection) / squaredNorm;
	auto p0Proj = segment0 + distance * segmentDirection;
	return p0Proj;
}

/// Check if the two triangles intersect using separating axis test.
/// Algorithm is implemented from http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tritri.pdf
///
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param t0v0,t0v1,t0v2 Vertices of the first triangle.
/// \param t1v0,t1v1,t1v2 Vertices of the second triangle.
/// \param t0n Normal of the first triangle, should be normalized.
/// \param t1n Normal of the second triangle, should be normalized.
/// \return True, if intersection is detected.
template <class T, int MOpt> inline
bool doesIntersectTriangleTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0n,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1n);

/// Check if the two triangles intersect using separating axis test.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param t0v0,t0v1,t0v2 Vertices of the first triangle.
/// \param t1v0,t1v1,t1v2 Vertices of the second triangle.
/// \return True, if intersection is detected.
template <class T, int MOpt> inline
bool doesIntersectTriangleTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2);

/// Calculate the contact between two triangles.
/// Algorithm presented in
/// https://docs.google.com/a/simquest.com/document/d/11ajMD7QoTVelT2_szGPpeUEY0wHKKxW1TOgMe8k5Fsc/pub.
/// If the triangle are known to intersect, the deepest penetration of the triangles into each other is calculated.
/// The triangle which penetrates less into the other triangle is chosen as contact.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param t0v0,t0v1,t0v2 Vertices of the first triangle.
/// \param t1v0,t1v1,t1v2 Vertices of the second triangle.
/// \param t0n Unit length normal of the first triangle, should be normalized.
/// \param t1n Unit length normal of the second triangle, should be normalized.
/// \param [out] penetrationDepth The depth of penetration.
/// \param [out] penetrationPoint0 The contact point on triangle0 (t0v0,t0v1,t0v2).
/// \param [out] penetrationPoint1 The contact point on triangle1 (t1v0,t1v1,t1v2).
/// \param [out] contactNormal The contact normal that points from triangle1 to triangle0.
/// \return True, if intersection is detected.
/// \note The [out] params are not modified if there is no intersection.
/// \note If penetrationPoint0 is moved by (contactNormal*penetrationDepth*0.5) and penetrationPoint1
/// is moved by -(contactNormal*penetrationDepth*0.5), the triangles will no longer be intersecting.
template <class T, int MOpt> inline
bool calculateContactTriangleTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0n,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1n,
	T* penetrationDepth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint0,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint1,
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal);

/// Calculate the contact between two triangles.
/// Algorithm presented in
/// https://docs.google.com/a/simquest.com/document/d/11ajMD7QoTVelT2_szGPpeUEY0wHKKxW1TOgMe8k5Fsc/pub.
/// If the triangle are known to intersect, the deepest penetration of the triangles into each other is calculated.
/// The triangle which penetrates less into the other triangle is chosen as contact.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param t0v0,t0v1,t0v2 Vertices of the first triangle, should be normalized.
/// \param t1v0,t1v1,t1v2 Vertices of the second triangle, should be normalized.
/// \param [out] penetrationDepth The depth of penetration.
/// \param [out] penetrationPoint0 The contact point on triangle0 (t0v0,t0v1,t0v2).
/// \param [out] penetrationPoint1 The contact point on triangle1 (t1v0,t1v1,t1v2).
/// \param [out] contactNormal The contact normal that points from triangle1 to triangle0.
/// \return True, if intersection is detected.
/// \note The [out] params are not modified if there is no intersection.
/// \note If penetrationPoint0 is moved by (contactNormal*penetrationDepth*0.5) and penetrationPoint1
/// is moved by -(contactNormal*penetrationDepth*0.5), the triangles will no longer be intersecting.
template <class T, int MOpt> inline
bool calculateContactTriangleTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	T* penetrationDepth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint0,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint1,
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal);

/// Calculate the contact between a capsule and a triangle.
/// If the shapes intersect, the deepest penetration of the capsule along the triangle normal is calculated.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param tv0,tv1,tv2 Vertices of the triangle.
/// \param tn Normal of the triangle, should be normalized.
/// \param cv0,cv1 Ends of the capsule axis.
/// \param cr Capsule radius.
/// \param [out] penetrationDepth The depth of penetration.
/// \param [out] penetrationPointTriangle The contact point on triangle.
/// \param [out] penetrationPointCapsule The contact point on capsule.
/// \param [out] contactNormal The contact normal that points from capsule to triangle.
/// \return True, if intersection is detected.
/// \note The [out] params are not modified if there is no intersection.
/// \note If penetrationPointTriangle is moved by (contactNormal*penetrationDepth*0.5) and penetrationPointCapsule
/// is moved by -(contactNormal*penetrationDepth*0.5), the shapes will no longer be intersecting.
template <class T, int MOpt> inline
bool calculateContactTriangleCapsule(
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
	const Eigen::Matrix<T, 3, 1, MOpt>& tn,
	const Eigen::Matrix<T, 3, 1, MOpt>& cv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& cv1,
	double cr,
	T* penetrationDepth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPointTriangle,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPointCapsule,
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal);

/// Calculate the contact between a capsule and a triangle.
/// If the shapes intersect, the deepest penetration of the capsule along the triangle normal is calculated.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param tv0,tv1,tv2 Vertices of the triangle.
/// \param cv0,cv1 Ends of the capsule axis.
/// \param cr Capsule radius.
/// \param [out] penetrationDepth The depth of penetration.
/// \param [out] penetrationPointTriangle The contact point on triangle.
/// \param [out] penetrationPointCapsule The contact point on capsule.
/// \param [out] contactNormal The contact normal that points from capsule to triangle.
/// \return True, if intersection is detected.
/// \note The [out] params are not modified if there is no intersection.
/// \note If penetrationPointTriangle is moved by (contactNormal*penetrationDepth*0.5) and penetrationPointCapsule
/// is moved by -(contactNormal*penetrationDepth*0.5), the shapes will no longer be intersecting.
template <class T, int MOpt> inline
bool calculateContactTriangleCapsule(
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
	const Eigen::Matrix<T, 3, 1, MOpt>& cv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& cv1,
	double cr,
	T* penetrationDepth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPointTriangle,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPointCapsule,
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal);

}; // namespace Math
}; // namespace SurgSim


#include "SurgSim/Math/TriangleCapsuleContactCalculation-inl.h"
#include "SurgSim/Math/TriangleTriangleIntersection-inl.h"
#include "SurgSim/Math/TriangleTriangleContactCalculation-inl.h"


#endif

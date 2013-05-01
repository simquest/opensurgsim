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
// 
// \TODO Local Definition for epsilon, remove epsilon as a parameter
// \TODO set return values to NAN
// \TODO return true or false, and pass by reference for all other numeric results
// \TODO use Eigen comparison functions for tests

#ifndef SURGSIM_GEOMETRY_H
#define SURGSIM_GEOMETRY_H

#include <SurgSim/Math/Vector.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace SurgSim 
{
namespace Math
{
	const static double DegenerateEpsilon = 1e-10;
	const static double IntersectionEpsilon = 1e-10;
	const static double ComparisonEpsilon = 1e-10;

/// Get the barycentric coordinates of a point with respect to a triangle
/// \param pt Vertex of the point.
/// \param tv0, tv1, tv2 Vertices of the triangle in counter clockwise order in respect to the normal.
/// \param tn Normal of the triangle (yes must be of norm 1 and a,b,c CCW).
/// \param [OUT] baryCoords Barycentric coordinates.
/// \param epsilon Precision required.
/// \return bool true on success; o.w., false and set the bary coords to 1/3
template <class T> inline
bool BaryCentricCoordinates(const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& pt,
							const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv0,
							const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv1,
							const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv2,
							const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tn,
							Eigen::Matrix<T, 3, 1, Eigen::DontAlign>* baryCoords)
{
	const T signedTriAreaX2 = ((tv1-tv0).cross(tv2-tv0)).dot(tn);
	if (signedTriAreaX2 < DegenerateEpsilon)
	{
		// SQ_ASSERT_WARNING(false, "Cannot compute barycentric coords (degenetrate triangle), assigning center");
		(*baryCoords) << (std::numeric_limits<double>::quiet_NaN()),
						 (std::numeric_limits<double>::quiet_NaN()),
						 (std::numeric_limits<double>::quiet_NaN());
		return false;
	}
	(*baryCoords)[0] = ((tv1-pt).cross(tv2-pt)).dot(tn) / signedTriAreaX2;
	(*baryCoords)[1] = ((tv2-pt).cross(tv0-pt)).dot(tn) / signedTriAreaX2;
	(*baryCoords)[2] = 1 - (*baryCoords)[0] - (*baryCoords)[1];
	return true;
}

/// Get the barycentric coordinates of a point with respect to a triangle.
/// Please note that each time you use this call the normal of the triangle will be 
/// calculated, if you convert more than one coordinate against this triangle, precalcualate
/// the normal and use the other version of this function
/// \param pt Vertex of the point.
/// \param tv0, tv1, tv2 Vertices of the triangle.
/// \param [OUT] baryCoords The Barycentric coordinates.
/// \param epsilon Precision required.
/// \return bool true on success; o.w., false and set the bary coords to 1/3
template <class T> inline
	bool BaryCentricCoordinates(const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& pt,
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv0,
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv1,
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv2,
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign>* baryCoords)
{
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign> tn = (tv1-tv0).cross(tv2-tv0);
	return BaryCentricCoordinates(pt, tv0, tv1, tv2, tn, baryCoords);
}

/// Calculate the normal distance between a point on a straight line
/// \param p		The input point.
/// \param v0,v1	Two vertices on the line.
/// \param [OUT]	pt The projection point.
/// \return			The normal distance of the point and the line
template <class T> inline
	T PointLineDistance(const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& p, 
						const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& v0, 
						const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& v1, 
						Eigen::Matrix<T, 3, 1, Eigen::DontAlign>* pt)
{	
	// The lines is parametrized by:
	//		q = v0 + lambda0 * (v1-v0)
	// and we solve for pq.v01 = 0;
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> v01 = v1-v0;
	T v01_norm2 = v01.squaredNorm();
	if ( v01_norm2 <= DegenerateEpsilon )
	{
		*pt = v0; // closest point is either
		T pv_norm2 = (p-v0).squaredNorm();
		return sqrt(pv_norm2);
	}	
	T lambda = (v01).dot(p-v0);
	*pt = v0 + lambda*v01/v01_norm2;
	return (*pt-p).norm();
}

/// Point segment distance, if the projection of the point is not within the segment, the
/// closest segment point is used for the distance calculation.
/// \param	p		  	The input point
/// \param	v0,v1	  	The segment extremities.
/// \param [out]	pt	Either the projection on the segment or one of the 2 vertices.
/// \param	epsilon   	The required precision.
/// \return				The distance of the point from the segment.
template <class T> inline
T PointSegDistance(const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& p, 
				   const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& v0,
				   const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& v1, 
				   Eigen::Matrix<T, 3, 1, Eigen::DontAlign>* pt)
{		
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> v01 = v1-v0;
	T v01_norm2 = v01.squaredNorm();
	if ( v01_norm2 <= DegenerateEpsilon )
	{
		*pt = v0; // closest point is either
		T pv_norm2 = (p-v0).squaredNorm();
		return sqrt(pv_norm2);
	}
	T lambda = v01.dot(p-v0);
	if ( lambda <= 0 )
	{
		*pt = v0;
	}
	else if ( lambda >= v01_norm2 )
	{
		*pt = v1;
	}
	else
	{
		*pt = v0 + lambda*v01/v01_norm2;
	}
	return (*pt-p).norm();
}

/// Determine the distance between two lines
/// \param l0v0, l0v1	Points on Line 0.
/// \param l1v0, l1v1	Points on Line 1.
/// \param [OUT] pt0	The closest point on line 0.
/// \param [OUT] pt1	The closest point on line 1.
/// \return				The normal distance between the two given lines 
/// \note We are using PointSegDistance for the degenerate cases as opposed to 
/// 	  PointLineDistance, why is that ??? (HS-2013-apr-26)
template <class T> inline
	T LineLineDistance(const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& l0v0, 
					   const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& l0v1, 
					   const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& l1v0, 
					   const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& l1v1, 
					   Eigen::Matrix<T, 3, 1, Eigen::DontAlign>* pt0,
					   Eigen::Matrix<T, 3, 1, Eigen::DontAlign>* pt1)
{
	// Based on the outline of http://www.geometrictools.com/Distance.html, also refer to 
	// http://geomalgorithms.com/a07-_distance.html for a geometric interpretation
	// The lines are parametrized by:
	//		p0 = l0v0 + lambda0 * (l0v1-l0v0)
	//		p1 = l1v0 + lambda1 * (l1v1-l1v0)
	// and we solve for p0p1 perpendicular to both lines
	T lambda0, lambda1;
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> l0v01 = l0v1-l0v0;
	T a = l0v01.dot(l0v01);
	if ( a <= DegenerateEpsilon )
	{ 
		// Degenerate line 0
		*pt0 = l0v0;
		return PointSegDistance(l0v0, l1v0, l1v1, pt1);		
	}
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> l1v01 = l1v1-l1v0;
	T b = -l0v01.dot(l1v01);
	T c = l1v01.dot(l1v01);
	if ( c <= DegenerateEpsilon )
	{ // Degenerate line 1
		*pt1 = l1v0;
		return PointSegDistance(l1v0, l0v0, l0v1, pt0);
	}
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> l0v0_l1v0 = l0v0-l1v0;
	T d = l0v01.dot(l0v0_l1v0);
	T e = -l1v01.dot(l0v0_l1v0);	
	T ratio = a*c-b*b;
	if ( ratio <= IntersectionEpsilon )
	{ 
		// parallel case
		lambda0 = 0;
		lambda1 = e / c;
	}
	else
	{
		// non-parallel case
		T inv_ratio = T(1) / ratio;
		lambda0 = ( b*e - c*d ) * inv_ratio;
		lambda1 = ( b*d - a*e ) * inv_ratio;
	}
	*pt0 = l0v0 + lambda0 * l0v01;
	*pt1 = l1v0 + lambda1 * l1v01;
	return ( (*pt0)-(*pt1)).norm();
}


/// Distance between two segments
/// \param l0v0, l0v1 Segment 0 Extremities.
/// \param l1v0, l1v1 Segment 1 Extremities.
/// \param [OUT] pt0 Closest point on segment 0
/// \param [OUT] pt1 Closest point on segment 1
/// \return Distance between the segments 
template <class T>
T SegSegDistance(
		const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& l0v0, 
		const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& l0v1, 
		const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& l1v0, 
		const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& l1v1, 
		Eigen::Matrix<T, 3, 1, Eigen::DontAlign>* pt0,
		Eigen::Matrix<T, 3, 1, Eigen::DontAlign>* pt1, 
		T epsilon)
{
	// Based on the outline of http://www.geometrictools.com/Distance.html, also refer to 
	// http://geomalgorithms.com/a07-_distance.html for a geometric interpretation
	// The segments are parametrized by:
	//		p0 = l0v0 + s * (l0v1-l0v0), with lambda0 between 0 and 1
	//		p1 = l1v0 + t * (l1v1-l1v0), with lambda1 between 0 and 1	
	// We are minimizing Q(s, t) = as*as + 2bst + ct*ct + 2ds + 2et + f,
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> l0v01 = l0v1-l0v0;
	T a = l0v01.dot(l0v01);
	if ( a <= epsilon )
	{ 
		// Degenerate segment 0
		*pt0 = l0v0;
		return PointSegDistance<T>(l0v0, l1v0, l1v1, pt1);	
	}
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> l1v01 = l1v1-l1v0;
	T b = -l0v01.dot(l1v01);
	T c = l1v01.dot(l1v01);
	if ( c <= epsilon )
	{ 
		// Degenerate segment 1
		*pt1 = l1v1;
		return PointSegDistance<T>(l1v0, l0v0, l0v1, pt0);	
	}
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> l0v0_l1v0 = l0v0-l1v0;
	T d = l0v01.dot(l0v0_l1v0);
	T e = -l1v01.dot(l0v0_l1v0);	
	T ratio = a*c-b*b;
	T s,t; // parametrization variables (do not initialize)
	int region = -1;
	T tmp;
	// Non-parallel case
	if ( abs(ratio) >= epsilon )
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
		s = b*e-c*d;
		t = b*d-a*e;
		if ( s >= 0 )
		{
			if ( s <= ratio )
			{
				if ( t >= 0 ) 
				{ 
					if ( t <= ratio ) 
						region = 0; 
					else 
						region = 3;
				}
				else 
				{ 
					region = 7;
				}
			}
			else
			{
				if ( t >= 0 ) 
				{ 
					if ( t <= ratio ) 
						region = 1;
					else 
						region = 2;
				}
				else 
				{ 
					region = 8;
				}
			}
		}
		else
		{
			if ( t >= 0 ) 
			{ 
				if ( t <= ratio ) 
					region = 5;
				else 
					region = 4;
			}
			else 
			{ 
				region = 6;
			}
		}		
		enum edge_type { s0, s1, t0, t1, edge_skip, edge_invalid };
		edge_type edge = edge_invalid;
		switch ( region )
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
			if ( a+b+d > 0 )
				edge = t1;
			else 
				edge = s1;
			break;
		case 3:
			edge = t1;
			break;
		case 4:
			// Q_s(0,1)/2 = b+d
			if ( b+d > 0 )
				edge = s0;
			else
				edge = t1;
			break;
		case 5:
			edge = s0;
			break;
		case 6:
			// Q_s(0,0)/2 = d
			if ( d > 0 )
				edge = s0;
			else
				edge = t0;
			break;
		case 7:
			edge = t0;
			break;
		case 8:
			// Q_s(1,0)/2 = a+d
			if ( a+d > 0 )
				edge = t0;
			else
				edge = s1;
			break;
		default:
			break;
		}
		switch ( edge )
		{		
		case s0:
			// F(t) = Q(0,t), F?(t) = 2*(e+c*t)
			// F?(T) = 0 when T = -e/c, then clamp between 0 and 1 (c always >= 0)
			s = 0;
			tmp = e;
			if ( tmp > 0 )
				t = 0;
			else if ( -tmp > c )
				t = 1;
			else
				t = -tmp/c;
			break;
		case s1:
			// F(t) = Q(1,t), F?(t) = 2*((b+e)+c*t)
			// F?(T) = 0 when T = -(b+e)/c, then clamp between 0 and 1 (c always >= 0)
			s = 1;
			tmp = b+e;		
			if ( tmp > 0 )
				t = 0;
			else if ( -tmp > c )
				t = 1;
			else
				t = -tmp/c;
			break;
		case t0:
			// F(s) = Q(s,0), F?(s) = 2*(d+a*s) => 
			// F?(S) = 0 when S = -d/a, then clamp between 0 and 1 (a always >= 0)
			t = 0;
			tmp = d;
			if ( tmp > 0 )
				s = 0;
			else if ( -tmp > a )
				s = 1;
			else
				s = -tmp/a;
			break;
		case t1:
			// F(s) = Q(s,1), F?(s) = 2*(b+d+a*s) => 
			// F?(S) = 0 when S = -(b+d)/a, then clamp between 0 and 1  (a always >= 0)
			t = 1;
			tmp = b+d;
			if ( tmp > 0 )
				s = 0;
			else if ( -tmp > a )
				s = 1;
			else
				s = -tmp/a;
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
		if ( b > 0 )
		{
			// Segments have different directions
			if ( d >= 0 )
			{
				// 0-0 end points since s-segment 0 less than t-segment 0
				s = 0;
				t = 0;
			}
			else if ( -d <= a )
			{
				// s-segment 0 end-point in the middle of the t 0-1 segment, get distance
				s = -d/a;
				t = 0;
			}
			else
			{
				// s-segment 1 is definitely closer
				s = 1;
				tmp = a+d;
				if ( -tmp >= b ) 
					t = 1; 
				else 
					t = -tmp/b;
			}
		}
		else
		{
			// Both segments have the same dir
			if ( -d >= a )
			{
				// 1-0
				s = 1;
				t = 0;
			}
			else if ( d <= 0 )
			{
				// mid-0
				s = -d/a;
				t = 0;
			}
			else
			{
				s = 0;
				// 1-mid
				if ( d >= -b ) 
					t = 1; 
				else 
					t = -d/b;
			}
		}
	}
	*pt0 = l0v0 + s * (l0v01);
	*pt1 = l1v0 + t * (l1v01);
	return ((*pt1)-(*pt0)).norm();
}

/// Calculate the distance of a point from a Triangle
/// \param pv0 Vertex of the point.
/// \param tv0, tv1, tv2 The vertices of the triangle
/// \param [OUT] Closest Point on the Trianle
/// \param epsilon Precision required
/// \return the distance between the point and the triangle
template <class T> inline
T PointTriDistance(const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& pv0,
				   const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv0,
				   const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv1,
				   const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv2, 
				   Eigen::Matrix<T, 3, 1, Eigen::DontAlign>* pt)
{
	// Based on the outline of http://www.geometrictools.com/Distance.html, also refer to 
	//	http://softsurfer.com/Archive/algorithm_0106 for a geometric interpretation 
	// The triangle is parametrized by:
	//		t: tv0 + s * (tv1-tv0) + t * (tv2-tv0) , with s and t between 0 and 1
	// We are minimizing Q(s, t) = as*as + 2bst + ct*ct + 2ds + 2et + f,
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> tv01 = tv1-tv0;
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> tv02 = tv2-tv0;
	T a = tv01.dot(tv01);
	if ( a <= DegenerateEpsilon )
	{ 
		// Degenerate edge 1		
		return PointSegDistance<T>(pv0, tv0, tv2, pt);	
	}
	T b = tv01.dot(tv02);
	T tCross = tv01.cross(tv02).squaredNorm();
	if ( tCross <= DegenerateEpsilon )
	{ 
		// Degenerate edge 2
		return PointSegDistance<T>(pv0, tv0, tv1, pt);	
	}
	T c = tv02.dot(tv02);
	if ( c <= DegenerateEpsilon )
	{ 
		// Degenerate edge 3
		return PointSegDistance<T>(pv0, tv0, tv1, pt);	
	}
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> tv0pv0 = tv0-pv0;
	T d = tv01.dot(tv0pv0);
	T e = tv02.dot(tv0pv0);
	T ratio = a*c-b*b;
	T s = b*e-c*d;
	T t = b*d-a*e;
	// Determine region (inside-outside triangle)
	int region = -1;
	if ( s+t <= ratio )
	{
		if ( s < 0 )
		{
			if ( t < 0 )
				region = 4;
			else
				region = 3;
		}
		else if ( t < 0 )
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
		if ( s < 0 )
			region = 2;
		else if ( t < 0 )
			region = 6;
		else
			region = 1;
	}
	//	Regions:
	//	    ^ t=0			
	//	 \ 2|			
	//	  \ |			
	//	   \|			
	//		\			
	//		|\			
	//		| \	  1	
	//	3	|  \				
	//		| 0 \		
	//	----|----\------->	s=0
	//		| 	  \	
	//	4	|	5  \   6		
	//		|	    \	
	//
	T numer, denom, tmp0, tmp1;
	enum edge_type { s0, t0, s1t1, edge_skip, edge_invalid };
	edge_type edge = edge_invalid;
	switch ( region )
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
		tmp0 = b+d;
		tmp1 = c+e;
		if ( tmp1 > tmp0 )
			edge = s1t1;
		else
			edge = s0;
		break;
	case 3:
		edge = s0;
		break;
	case 4:
		// Grad(Q(0,0)).(0,1)/2 = e
		// Grad(Q(0,0)).(1,0)/2 = d
		if ( e >= d )
			edge = t0;
		else
			edge = s0;
		break;
	case 5:
		edge = t0;
		break;
	case 6:
		// Grad(Q(1,0)).(-1,0)/2 = -a-d 
		// Grad(Q(1,0)).(-1,1)/2 = -a-d+b+e 
		tmp0 = -a-d;
		tmp1 = -a-d+b+e;
		if ( tmp1 > tmp0 )
			edge = t0;
		else
			edge = s1t1;
		break;
	default:
		break ;
	}
	switch ( edge )
	{
	case s0:
		// F(t) = Q(0, t), F'(t)=0 when -e/c = 0
		s = 0;
		if ( e >= 0 )
			t = 0;
		else
			t = ( -e >= c ? 1 : -e/c );
		break;
	case t0:
		// F(s) = Q(s, 0), F'(s)=0 when -d/a = 0
		t = 0;
		if ( d >= 0 )
			s = 0;
		else
			s = ( -d >= a ? 1 : -d/a );
		break;
	case s1t1:
		// F(s) = Q(s, 1-s), F'(s) = 0 when (c+e-b-d)/(a-2b+c) = 0 (denom = || tv01-tv02 ||^2 always > 0)
		numer = c+e-b-d;
		if ( numer <= 0 )
		{
			s = 0;
		}
		else
		{
			denom = a-2*b+c;
			s = ( numer >= denom ? 1 : numer / denom );
		}
		t = 1-s;
		break;
	case edge_skip:
		break;
	default:
		break ;
	}
	*pt = tv0 + s * tv01 + t * tv02;
	return ( (*pt)-pv0).norm();
}


/// Quick check to see if a point is inside a triangle - use BaryCentricCoordinates is coords are required
/// \param pt Vertex of the point.
/// \param tv0, tv1, tv2 Vertices of the triangle, must be in CCW.
/// \param tn Normal of the triangle (yes must be of norm 1 and a,b,c CCW).
/// \param epsilon Required precision.
/// \return true if pt lies inside the triangle tv0, tv1, tv2, false otherwise.

template <class T> inline
bool PointInsideTriangle(const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& pt, 
						 const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv0, 
						 const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv1, 
						 const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv2, 
						 const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tn)
{
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> baryCoords;
	bool result = BaryCentricCoordinates(pt, tv0, tv1, tv2, tn, &baryCoords);
	return ( result && 
			baryCoords[0] >= -ComparisonEpsilon && 
			baryCoords[1] >= -ComparisonEpsilon && 
			baryCoords[2] >= -ComparisonEpsilon);
}


/// Quick check to see if a point is inside a triangle - use BaryCentricCoordinates is coordinates is, required.
/// Please note that the normal will be calculated each time you use this call, if you are doing more than one
/// test with the same triangle, precalcluate the normal and pass it.
/// \param pt Vertex of the point.
/// \param tv0, tv1, tv2 Vertices of the triangle, must be in CCW.
/// \param epsilon Required precision.
/// \return true if pt lies inside the triangle tv0, tv1, tv2, false otherwise.
template <class T> inline
bool PointInsideTriangle(
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& pt, 
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv0, 
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv1, 
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv2)
{
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> baryCoords;
	bool result = BaryCentricCoordinates(pt, tv0, tv1, tv2, &baryCoords);
	return (result && baryCoords[0] >= -ComparisonEpsilon && 
			baryCoords[1] >= -ComparisonEpsilon && 
			baryCoords[2] >= -ComparisonEpsilon);
}


/// Get the intersection of a line segment with a triangle, if the segment intersects this will return the 
/// penetration depth on the intersection. 
/// \param v0,v1 Extremities of the segment.
/// \param tv0,tv1,tv2 The triangle vertices.
/// \param [OUT] intersection The point where the triangle and the line segment intersect.
/// \param epsilon The required precision.
/// \return 1 for no collision, o.w. either -1 or the penetration depth of the segment (i.e (intersection-v0).norm())
/// \note HS-2013-apr-29 I think this should be rewritten as returning false or true and passing the
/// 	  distance as a second parameter ...
template <class T> inline
T SegTriCollide(
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& v0,
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& v1,
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv0, 
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv1, 
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tv2,
	const Eigen::Matrix<T, 3, 1, Eigen::DontAlign>& tn,
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign>* intersection)
{
	// From http://geomalgorithms.com/a06-_intersect-2.html#intersect_RayTriangle
	// Triangle edges vectors
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> u = tv1-tv0;
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> v = tv2-tv0;
	// Ray direction vector
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> dir = v1-v0;
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> w0 = v0-tv0;
	T a = -tn.dot(w0);
	T b = tn.dot(dir);
	// Ray is parallel to triangle plane
	if ( fabs(b) <= IntersectionEpsilon )
	{
		if ( a == 0 )
		{
			// Ray lies in triangle plane
			Eigen::Matrix<T, 3, 1, Eigen::DontAlign> baryCoords;
			for ( int i=0; i<2; ++i )
			{
				BaryCentricCoordinates((i==0?v0:v1), tv0, tv1, tv2, tn, &baryCoords);
				if ( baryCoords[0] >= 0 && baryCoords[1] >= 0 && baryCoords[2] >= 0 ) 
				{
					return 0;
				}
			}
		}
		else 
		{
			// Ray disjoint from plane
			return 1;
		}
	}
	// Get intersect point of ray with triangle plane
	T r = a / b;
	// Ray goes away from triangle
	if ( r < 0 || r > 1 )
		return 1;
	// Intersect point of ray and plane
	*intersection = v0 + r * dir;
	// Collision point inside T?
	T uu = u.dot(u);
	T uv = u.dot(v);
	T vv = v.dot(v);
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> w = (*intersection) - tv0;
	T wu = w.dot(u);
	T wv = w.dot(v);
	T D = uv * uv - uu * vv;
	// Get and test parametric coords
	T s = (uv * wv - vv * wu) / D;
	// I is outside T
	if ( s < 0 || s > 1 )
	{
		return 1;
	}
	T t = ( uv * wu - uu * wv ) / D;
	// I is outside T
	if ( t < 0 || (s + t) > 1 )
		return 1;
	// I is in T
	return -( (*intersection)-v1).norm();
}

}; // namespace Math
}; // namespace SurgSim


#endif
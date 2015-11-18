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

#include "SurgSim/Collision/SegmentSegmentCcdStaticContact.h"
#include "SurgSim/Math/Scalar.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

SegmentSegmentCcdStaticContact::SegmentSegmentCcdStaticContact(): m_degenerateEpsilon(1.0e-18)
{
}

bool SegmentSegmentCcdStaticContact::collideStaticSegmentSegment(
	const std::array<SurgSim::Math::Vector3d, 2>& p,
	const std::array<SurgSim::Math::Vector3d, 2>& q,
	double distanceEpsilon,
	double* r, double* s)
{
	return collideStaticSegmentSegment(p, q, distanceEpsilon / 2.0, distanceEpsilon / 2.0, r, s);
}

bool SegmentSegmentCcdStaticContact::collideStaticSegmentSegment(
	const std::array<SurgSim::Math::Vector3d, 2>& p,
	const std::array<SurgSim::Math::Vector3d, 2>& q,
	double radiusP, double radiusQ,
	double* r, double* s)
{
	const double totalThickness = radiusP + radiusQ;
	const double totalThickness2 = totalThickness * totalThickness;

	// Based on the outline of:
	// https://www.assembla.com/spaces/OpenSurgSim/documents/cRWomWC2er5ykpacwqjQYw/download/cRWomWC2er5ykpacwqjQYw.
	// We are minimizing the squared distance R(sp,sq) = (a.sp)^2 + 2b.sp.sq + (c.sq)^2 + 2d.sq + 2e.sp + f
	// for P(s) = (1-s).P0 + s.P1 and Q(t) = (1-t).Q0 + t.Q1, and as defined in the paper:
	//  a = (P1 - P0)(P1 - P0)
	//  b = -(P1 - P0)(Q1 - Q0)
	//  c = (Q1 - Q0)(Q1 - Q0)
	//  d = (P1 - P0)(P0 - Q0)
	//  e = -(Q1 - Q0)(P0 - Q0)
	//  f = (P0 - Q0)(P0 - Q0)

	// First determine if either of our segments are really points. If so this is easier. Take advantage.
	auto p0p1 = p[1] - p[0];
	double a = p0p1.dot(p0p1);
	if (a <= m_degenerateEpsilon)   // Degenerate segment P
	{
		*r = 0.0;
		return collideStaticPointSegment(p[0], q, radiusP, radiusQ, s);
	}

	auto q0q1 = q[1] - q[0];
	double c = q0q1.dot(q0q1);
	if (c <= m_degenerateEpsilon)   // Degenerate segment Q
	{
		*s = 0;
		return collideStaticPointSegment(q[0], p, radiusQ, radiusP, r);
	}

	double b = -p0p1.dot(q0q1);
	auto q0p0 = p[0] - q[0];
	double d = p0p1.dot(q0p0);
	double e = -q0q1.dot(q0p0);
	double ratio = a * c - b * b;

	// Characterize the angle between p and q. If it is big enough
	// then handle it with the generic code. Otherwise, use an
	// algorithm specific to parallel segments.
	if (ratio >= m_degenerateEpsilon)
	{
		// This section of the code carries out the steps of the cited paper.
		//   1. Determine the points of closest approach for the infinite lines containing p and q
		//   2. Determine where these values fall with respect to the segment end points and which edges (if any)
		//      must be clamped to the parametric range [0.0, 1.0]
		//   3. Calculate the parametrics for the closest approach of the segments using the edge information.
		double infiniteLineR = b * e - c * d;
		double infiniteLineS = b * d - a * e;
		*r = infiniteLineR;
		*s = infiniteLineS;
		SegmentCcdEdgeType edge = computeCollisionEdge(a, b, d, infiniteLineR, infiniteLineS, ratio);
		computeCollisionParametrics(edge, a, b, c, d, e, ratio, r, s);
	}
	else // Parallel case
	{
		computeParallelSegmentParametrics(a, b, d, r, s);
	}

	SURGSIM_ASSERT(*r >= 0.0 && *r <= 1.0) << "Segment collision s should be in [0,1]!";
	SURGSIM_ASSERT(*s >= 0.0 && *s <= 1.0) << "Segment collision s should be in [0,1]!";

	Vector3d pBar = Math::interpolate(p[0], p[1], *r);
	Vector3d qBar = Math::interpolate(q[0], q[1], *s);

	Vector3d pq = qBar - pBar;
	return pq.squaredNorm() <= totalThickness2;
}

bool SegmentSegmentCcdStaticContact::collideStaticPointSegment(
	const Math::Vector3d& point,
	const std::array<SurgSim::Math::Vector3d, 2>& p,
	double thicknessPoint, double thicknessSegment,
	double* r)
{
	Math::Vector3d b = p[0];
	Math::Vector3d c = p[1];
	auto ba = point - b;
	auto ca = point - c;
	auto bc = c - b;
	double baNormSQ = ba.squaredNorm();
	double caNormSQ = ca.squaredNorm();
	double bcNormSQ = bc.squaredNorm();

	double totalThicknessSQ = (thicknessPoint + thicknessSegment) * (thicknessPoint + thicknessSegment);

	// p is essentially a point
	if (bcNormSQ < m_degenerateEpsilon)
	{
		if (baNormSQ <= totalThicknessSQ)
		{
			*r = 0.0;
			return true;
		}
		if (caNormSQ <= totalThicknessSQ)
		{
			*r = 1.0;
			return true;
		}
		else
		{
			return false;
		}
	}

	// b!=c => compute the projection abscissa
	*r =  bc.dot(ba) / bcNormSQ;

	// Clamp abscissa
	Math::clamp(r, 0.0, 1.0, 0.0);

	// Compute the closest point of a on [bc]
	Math::Vector3d closestPtOnBC = Math::interpolate(b, c, *r);
	return (closestPtOnBC - point).squaredNorm() <= totalThicknessSQ;
}

SegmentSegmentCcdStaticContact::SegmentCcdEdgeType SegmentSegmentCcdStaticContact::computeCollisionEdge(
	double a, double b, double d,
	double r, double s, double ratio) const
{
	// Region mappings from reference:
	//
	//	    r=0    r=1
	//		^
	//      |       |
	//	4   |   3   |   2
	//	----|-------|-------    s=1
	//	    |		|
	//	5   |   0   |   1
	//	    |       |
	//	----|-------|------->   s=0
	//	    |       |
	//  6   |   7   |   8
	//      |       |
	//
	SegmentSegmentCcdStaticContact::SegmentCcdEdgeType edge = SegmentCcdEdgeTypeEdgeInvalid;

	if (r >= 0)
	{
		if (r <= ratio)
		{
			if (s >= 0)
			{
				if (s <= ratio)
				{
					// region = 0; (0 <= r,s <= 1)
					edge = SegmentCcdEdgeTypeEdgeSkip;
				}
				else
				{
					// region = 3; (0 <= r <= 1; 1 <= s)
					edge = SegmentCcdEdgeTypeS1;
				}
			}
			else
			{
				// region = 7; (0 <= r <= 1; s <= 0)
				edge = SegmentCcdEdgeTypeS0;
			}
		}
		else
		{
			if (s >= 0)
			{
				if (s <= ratio)
				{
					// region = 1; (1 <= r; 0 <= s <= 1)
					edge = SegmentCcdEdgeTypeR1;
				}
				else
				{
					// region = 2; (1 <= r,s)
					if (a + b + d > 0)
					{
						edge = SegmentCcdEdgeTypeS1;
					}
					else
					{
						edge = SegmentCcdEdgeTypeR1;
					}
				}
			}
			else
			{
				// region = 8; (1 <= r; s <= 0)
				if (a + d > 0)
				{
					edge = SegmentCcdEdgeTypeS0;
				}
				else
				{
					edge = SegmentCcdEdgeTypeR1;
				}
			}
		}
	}
	else
	{
		if (s >= 0)
		{
			if (s <= ratio)
			{
				// region = 5;(r <= 0; 0 <= s <= 1)
				edge = SegmentCcdEdgeTypeR0;
			}
			else
			{
				// region = 4;  (r <= 0; 1 <= s)
				if (b + d > 0)
				{
					edge = SegmentCcdEdgeTypeR0;
				}
				else
				{
					edge = SegmentCcdEdgeTypeS1;
				}
			}
		}
		else
		{
			// region = 6; (r <= 0; s <= 0)
			if (d > 0)
			{
				edge = SegmentCcdEdgeTypeR0;
			}
			else
			{
				edge = SegmentCcdEdgeTypeS0;
			}
		}
	}
	return edge;
}

void SegmentSegmentCcdStaticContact::computeCollisionParametrics(SegmentCcdEdgeType edge,
		double a, double b, double c, double d, double e,
		double ratio, double* r, double* s) const
{
	// On entry r and s are parametrically calculated based on infinite lines, i.e., r and s may not lie in [0, 1]
	// *r = b * e - c * d
	// *s = b * d - a * e
	double tmp;
	switch (edge)
	{
		case SegmentCcdEdgeTypeR0:
			// F(s) = Q(0,s), F?(s) = 2*(e+c*s)
			// F?(T) = 0 when T = -e/c, then clamp between 0 and 1 (c always >= 0)
			*r = 0.0;
			tmp = -e;
			if (tmp < 0)
			{
				*s = 0;
			}
			else if (tmp > c)
			{
				*s = 1.0;
			}
			else
			{
				*s = tmp / c;
			}
			break;
		case SegmentCcdEdgeTypeR1:
			// F(s) = Q(1,s), F?(s) = 2*((b+e)+c*s)
			// F?(T) = 0 when T = -(b+e)/c, then clamp between 0 and 1 (c always >= 0)
			*r = 1.0;
			tmp = -b - e;
			if (tmp < 0)
			{
				*s = 0.0;
			}
			else if (tmp > c)
			{
				*s = 1.0;
			}
			else
			{
				*s = tmp / c;
			}
			break;
		case SegmentCcdEdgeTypeS0:
			// F(r) = Q(r,0), F?(r) = 2*(d+a*r) =>
			// F?(S) = 0 when S = -d/a, then clamp between 0 and 1 (a always >= 0)
			*s = 0.0;
			tmp = -d;
			if (tmp < 0)
			{
				*r = 0.0;
			}
			else if (tmp > a)
			{
				*r = 1.0;
			}
			else
			{
				*r = tmp / a;
			}
			break;
		case SegmentCcdEdgeTypeS1:
			// F(r) = Q(r,1), F?(r) = 2*(b+d+a*r) =>
			// F?(S) = 0 when S = -(b+d)/a, then clamp between 0 and 1  (a always >= 0)
			*s = 1.0;
			tmp = -b - d;
			if (tmp < 0.0)
			{
				*r = 0.0;
			}
			else if (tmp > a)
			{
				*r = 1.0;
			}
			else
			{
				*r = tmp / a;
			}
			break;
		case SegmentCcdEdgeTypeEdgeSkip:
			tmp = 1.0 / ratio;
			*r *= tmp;
			*s *= tmp;
			break;
		default:
			break;
	}
}

void SegmentSegmentCcdStaticContact::computeParallelSegmentParametrics(double a, double b, double d, double* r,
		double* s) const
{
	if (b > 0.0)
	{
		// Segments have different directions
		if (d >= 0.0)
		{
			// 0-0 end points since r-segment 0 less than s-segment 0
			*r = 0.0;
			*s = 0.0;
		}
		else if (-d <= a)
		{
			// s-segment 0 end-point in the middle of the r 0-1 segment, get distance
			*r = -d / a;
			*s = 0.0;
		}
		else
		{
			// r-segment 1 is definitely closer
			*r = 1.0;
			double tmp = a + d;
			if (-tmp >= b)
			{
				*s = 1.0;
			}
			else
			{
				*s = -tmp / b;
			}
		}
	}
	else
	{
		// Both segments have the same dir
		if (-d >= a)
		{
			// 1-0
			*r = 1.0;
			*s = 0.0;
		}
		else if (d <= 0.0)
		{
			// mid-0
			*r = -d / a;
			*s = 0.0;
		}
		else
		{
			*r = 0.0;
			// 1-mid
			if (d >= -b)
			{
				*s = 1.0;
			}
			else
			{
				*s = -d / b;
			}
		}
	}
}

}; // namespace Collision
}; // namespace SurgSim

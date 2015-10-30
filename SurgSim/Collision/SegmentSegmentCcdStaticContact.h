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

#ifndef SURGSIM_COLLISION_SEGMENTSEGMENTCCDSTATICCONTACT_H
#define SURGSIM_COLLISION_SEGMENTSEGMENTCCDSTATICCONTACT_H

#include <array>

#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Collision
{

/// SegmentSegmentCcdStaticContact computes if there is contact between two segments at a specific point in time
/// in support of the CCD calculations for moving intervals. Algorithm optimizations improve performance for
/// specific orientations and positions of segments such as parallel segments, or segments where the closest
/// approach is at one or both of the segment endpoints.
///
/// \sa SegmentSegmentCcdIntervalCheck
///
class SegmentSegmentCcdStaticContact
{
public:
	enum SegmentCcdEdgeType
	{
		SegmentCcdEdgeTypeR0,			// Closest approach occurs at parametric value r = 0
		SegmentCcdEdgeTypeR1,			// Closest approach occurs at parametric value r = 1
		SegmentCcdEdgeTypeS0,			// Closest approach occurs at parametric value s = 0
		SegmentCcdEdgeTypeS1,			// Closest approach occurs at parametric value s = 1
		SegmentCcdEdgeTypeEdgeSkip,		// Closest approach is not at segment boundary (0.0 <= r,s, <= 1.0)
		SegmentCcdEdgeTypeEdgeInvalid	// Invalid value
	};

	/// Constructor.
	SegmentSegmentCcdStaticContact();

	/// Determine whether two "zero radius" segments collide. For moving segments, this represents contact at a
	/// specific point in time.
	/// \param p segment 1 endpoints.
	/// \param q segment 2 endpoints.
	/// \param distanceEpsilon closeness parameter for the zero thickness collision.
	/// \param r [out] parametric location of the collision point (if any) on segment 1.
	/// \param s [out] parametric location of the collision point (if any) on segment 2.
	/// \return false if no collision is occurring, or true otherwise.
	bool collideStaticSegmentSegment(
		const std::array<SurgSim::Math::Vector3d, 2>& p,
		const std::array<SurgSim::Math::Vector3d, 2>& q,
		double distanceEpsilon,
		double* r, double* s);

	/// Determine whether two thick segments collide. For moving segments, this represents contact at a
	/// specific point in time.
	/// \param p segment 1 endpoints.
	/// \param q segment 2 endpoints.
	/// \param radiusP thickness of segment 1.
	/// \param radiusQ thickness of segment 2.
	/// \param r [out] parametric location of the collision point (if any) on segment 1.
	/// \param s [out] parametric location of the collision point (if any) on segment 2.
	/// \return false if no collision is occurring, or true otherwise.
	bool collideStaticSegmentSegment(
		const std::array<SurgSim::Math::Vector3d, 2>& p,
		const std::array<SurgSim::Math::Vector3d, 2>& q,
		double radiusP, double radiusQ,
		double* r, double* s);

protected:
	/// Determine whether a single point and a segment collide.
	/// \param point point position.
	/// \param p segment endpoints.
	/// \param thicknessPoint radius of the point.
	/// \param thicknessSegment radius of the segment.
	/// \param r [out] parametric location of the collision point (if any) on segment p.
	/// \return false if no collision is occurring, or true otherwise.
	bool collideStaticPointSegment(
		const Math::Vector3d& point,
		const std::array<SurgSim::Math::Vector3d, 2>& p,
		double thicknessPoint, double thicknessSegment,
		double* r
	);

	/// Find the edge to be clamped for the closest point solution using the outline of:
	/// https://www.assembla.com/spaces/OpenSurgSim/documents/cRWomWC2er5ykpacwqjQYw/download/cRWomWC2er5ykpacwqjQYw
	///
	/// Calculates the parametric value that must be clamped in determining the segment -
	/// segment distance where:
	/// SegmentCcdEdgeTypeR0 clamp parametric value r to 0
	/// SegmentCcdEdgeTypeR1 clamp parametric value r to 1
	/// SegmentCcdEdgeTypeS0 clamp parametric value s to 0
	/// SegmentCcdEdgeTypeS1 clamp parametric value s to 1
	/// SegmentCcdEdgeSkip both values are with [0, 1]
	/// a = (P1 - P0)(P1 - P0)
	/// b = -(P1 - P0)(Q1 - Q0)
	/// c = (Q1 - Q0)(Q1 - Q0)
	/// d = (P1 - P0)(P0 - Q0)
	/// e = -(Q1 - Q0)(P0 - Q0)
	/// f = (P0 - Q0)(P0 - Q0)
	/// \param a value of p dot p
	/// \param b value of -(p dot q)
	/// \param d value of p dot (q[0] - p[0])
	/// \param r unnormalized parametric location of the intersection point on line p
	/// \param s unnormalized parametric location of the intersection point on line q
	/// \param ratio normalization value defined as (p dot p) . (q dot q) - (p dot q)^2.
	/// \return an indicator of the edge (r and s) which must be clamped and its clamp value.
	SegmentCcdEdgeType computeCollisionEdge(double a, double b, double d,
											double r, double s, double ratio) const;

	/// Given an edge indicator, clamp the indicated parametric edge and calculate the minimum parametric
	/// value for the other segment using the outline of:
	/// https://www.assembla.com/spaces/OpenSurgSim/documents/cRWomWC2er5ykpacwqjQYw/download/cRWomWC2er5ykpacwqjQYw
	/// Definitions of the values are:
	/// a = (P1 - P0)(P1 - P0)
	/// b = -(P1 - P0)(Q1 - Q0)
	/// c = (Q1 - Q0)(Q1 - Q0)
	/// d = (P1 - P0)(P0 - Q0)
	/// e = -(Q1 - Q0)(P0 - Q0)
	/// f = (P0 - Q0)(P0 - Q0)
	/// \param edge indicator of previously calculated edge constraint
	/// \param a value of p dot p
	/// \param b value of -(p dot q)
	/// \param c value of q dot q
	/// \param d value of p dot (p[0] - q[0])
	/// \param e value of -(q dot (p[0] - q[0]))
	/// \param ratio normalization value defined as (p dot p) . (q dot q) - (p dot q)^2.
	/// \param r [out] parametric location of the intersection point on segment p
	/// \param s [out] parametric location of the intersection point on segment q
	void computeCollisionParametrics(SegmentCcdEdgeType edge, double a, double b, double c, double d, double e,
									 double ratio, double* r, double* s) const;

	/// Calculate the parametric values that give the minimum distance for two parallel segments
	/// value for the other edge. Definitions of the values are:
	/// a = (P1 - P0)(P1 - P0)
	/// b = -(P1 - P0)(Q1 - Q0)
	/// c = (Q1 - Q0)(Q1 - Q0)
	/// d = (P1 - P0)(P0 - Q0)
	/// e = -(Q1 - Q0)(P0 - Q0)
	/// f = (P0 - Q0)(P0 - Q0)
	/// \param a value of p dot p
	/// \param b value of -(p dot q)
	/// \param d value of p dot (p[0] - q[0])
	/// \param r [out] parametric location of the intersection point on segment p
	/// \param s [out] parametric location of the intersection point on segment q
	void computeParallelSegmentParametrics(double a, double b, double d, double* r, double* s) const;

private:
	/// During collision, points closer than this value are considered a single point
	const double m_degenerateEpsilon;
};

}; // namespace Collision
}; // namespace SurgSim

#endif // SURGSIM_COLLISION_SEGMENTSEGMENTCCDSTATICCONTACT_H

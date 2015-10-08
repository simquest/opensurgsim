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

/// SegmentSegmentCcdStaticContact computes the self collisions among a SegmentMesh under motion at two
/// time points parametrized over the time interval [0,1]. An initial phase uses the AABB tree to
/// select a set of potentially colliding segments from the SegmentMesh. For each of these
/// candidate segment pairs, the goal is to determine the point of earliest contact should any exist.
///
/// At the highest level the actual collision detection of candidate segment pairs is a two phase
/// algorithm. First determine if there is contact at the start of an interval and report the contact if
/// found. If no contact is found at the start, subdivide the interval, determine which of the resulting
/// candidate subintervals may have collisions, and then recursively check those promising subintervals.
/// Note that a simple algorithm based on interval arithmetic (including the Interval, LinearMotion and
/// Polynomial interval classes) allows for a quick determination of promising subintervals allowing many
/// of the subintervals to be pruned during the subdivision step without forcing the recursion to bottom out.
///
/// \sa Interval, LinearMotion, Polynomial, SegmentSegmentCcdIntervalCheck
///
class SegmentSegmentCcdStaticContact
{
public:
	enum SegmentCcdEdgeType
	{
		SegmentCcdEdgeTypeR0,
		SegmentCcdEdgeTypeR1,
		SegmentCcdEdgeTypeS0,
		SegmentCcdEdgeTypeS1,
		SegmentCcdEdgeTypeEdgeSkip,
		SegmentCcdEdgeTypeEdgeInvalid
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
	/// \param thicknessA radius of A.
	/// \param thicknessP radius of segment P.
	/// \param r [out] parametric location of the collision point (if any) on segment p.
	/// \return false if no collision is occurring, or true otherwise.
	bool collideStaticPointSegment(
		const Math::Vector3d& point,
		const std::array<SurgSim::Math::Vector3d, 2>& p,
		double thicknessA, double thicknessP,
		double* r
	);

	/// Get the region of the global minimum in the r-s space based on the line-line solution
	/// for parametric variables r and s. The regions determine the potential solutions for
	/// the contact problem. Region 0 is the proper collision region where the parametric
	/// values of r and s are in the range [0, 1]. For all other regions, at least one of
	/// the parametric values will need to be clamped to the [0, 1] range.
	///
	///		r=0	   r=1
	///		^
	///		|		|
	///	4	|	3	|	2
	///	----|-------|-------	s=1
	///		|		|
	///	5	|	0	|	1
	///		|		|
	///	----|-------|------->	s=0
	///		|		|
	///	6	|	7	|	8
	///		|		|
	///
	/// \param r unnormalized parametric location of the intersection point on line p
	/// \param s unnormalized parametric location of the intersection point on line q
	/// \param ratio normalization value defined as (p dot p) . (q dot q) - (p dot q)^2.
	/// \return the region where r and s lie as defined in the above diagram
	int computeCollisionRegion(double r, double s, double ratio) const;

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
	/// \param region region under consideration
	/// \param a value of p dot p
	/// \param b value of -(p dot q)
	/// \param d value of p dot (q[0] - p[0])
	/// \return an indicator of the edge (r and s) which must be clamped and its clamp value.
	SegmentCcdEdgeType computeCollisionEdge(int region, double a, double b, double d) const;

	/// Given an edge indicator, clamp the indicated parametric edge and calculate the minimum parametric
	/// value for the other edge. Definitions of the values are:
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

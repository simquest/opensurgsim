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

#ifndef SURGSIM_COLLISION_SEGMENTSEGMENTCCDMOVINGCONTACT_H
#define SURGSIM_COLLISION_SEGMENTSEGMENTCCDMOVINGCONTACT_H

#include <array>

#include "SurgSim/Collision/SegmentSegmentCcdIntervalCheck.h"
#include "SurgSim/Collision/SegmentSegmentCcdStaticContact.h"

namespace SurgSim
{
namespace Math
{
class SegmentMeshShape;
};

namespace Collision
{

class CollisionPair;

/// SegmentSegmentCcdMovingContact computes the self collisions among a SegmentMesh under motion at two
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
class SegmentSegmentCcdMovingContact
{
public:
	/// Constructor.
	SegmentSegmentCcdMovingContact();

	/// Calculate if, where, and when the segments p and q collide in the interval from t = 0 to t = 1
	/// for "zero" thickness segments. This case is essentially an upper level wrapper. It delegates the detection
	/// to collideSegmentSegmentBaseCase and then adds whatever contacts are generated to the collision
	/// pair list.
	/// \param pt0Positions are the segment endpoints for the first segment at time t=0.
	/// \param pt1Positions are the segment endpoints for the first segment at time t=1.
	/// \param qt0Positions are the segment endpoints for the second segment at time t=0.
	/// \param qt1Positions are the segment endpoints for the second segment at time t=1.
	/// \param thicknessEpsilon spatial nearness criteria for declaring a contact.
	/// \param timePrecisionEpsilon time nearness criteria for declaring a contact.
	/// \param t [out] parametric location of the collision along the time axes in the interval [0, 1]
	/// \param r [out] parametric location of the collision along p in the interval [0, 1]
	/// \param s [out] parametric location of the collision along q in the interval [0, 1]
	/// \param pToQDir [out] direction from the contact point on p to the contact point on q
	/// \return true if p and q collide in interval [0, 1]
	bool collideMovingSegmentSegment(
		const std::array<Math::Vector3d, 2>& pt0Positions,
		const std::array<Math::Vector3d, 2>& pt1Positions,
		const std::array<Math::Vector3d, 2>& qt0Positions,
		const std::array<Math::Vector3d, 2>& qt1Positions,
		double thicknessEpsilon,
		double timePrecisionEpsilon,
		double* t, double* r, double* s, Math::Vector3d* pToQDir);

	/// Calculate if, where, and when the segments p and q collide in the interval from t = 0 to t = 1
	/// for thick segments. This case is essentially an upper level wrapper. It delegates the detection
	/// to collideSegmentSegmentBaseCase and then adds whatever contacts are generated to the collision
	/// pair list.
	/// \param pt0Positions are the segment endpoints for the first segment at time t=0.
	/// \param pt1Positions are the segment endpoints for the first segment at time t=1.
	/// \param qt0Positions are the segment endpoints for the second segment at time t=0.
	/// \param qt1Positions are the segment endpoints for the second segment at time t=1.
	/// \param thicknessP radius of segment p.
	/// \param thicknessQ radius of segment q.
	/// \param timePrecisionEpsilon time nearness criteria for declaring a contact.
	/// \param t [out] parametric location of the collision along the time axes in the interval [0, 1]
	/// \param r [out] parametric location of the collision along p in the interval [0, 1]
	/// \param s [out] parametric location of the collision along q in the interval [0, 1]
	/// \param pToQDir [out] direction from the contact point on p to the contact point on q
	/// \return true if p and q collide in interval [0, 1]
	bool collideMovingSegmentSegment(
		const std::array<Math::Vector3d, 2>& pt0Positions,
		const std::array<Math::Vector3d, 2>& pt1Positions,
		const std::array<Math::Vector3d, 2>& qt0Positions,
		const std::array<Math::Vector3d, 2>& qt1Positions,
		double thicknessP,
		double thicknessQ,
		double timePrecisionEpsilon,
		double* t, double* r, double* s, Math::Vector3d* pToQDir);

protected:
	/// Manage the collision of moving segments as a series of cases based on the segment
	/// relationships over the moving interval.
	/// \param pT0 are the segment endpoints for the first segment at time t=0.
	/// \param pT1 are the segment endpoints for the first segment at time t=1.
	/// \param qT0 are the segment endpoints for the second segment at time t=0.
	/// \param qT1 are the segment endpoints for the second segment at time t=1.
	/// \param thicknessP radius of segment p.
	/// \param thicknessQ radius of segment q.
	/// \param timePrecisionEpsilon time nearness criteria for declaring a contact.
	/// \param t [out] parametric location of the collision along the time axes in the interval [0, 1]
	/// \param r [out] parametric location of the collision along p in the interval [0, 1]
	/// \param s [out] parametric location of the collision along q in the interval [0, 1]
	/// \return true if p and q collide in interval [0, 1]
	bool collideSegmentSegmentBaseCase(
		const std::array<Math::Vector3d, 2>& pT0,
		const std::array<Math::Vector3d, 2>& pT1,
		const std::array<Math::Vector3d, 2>& qT0,
		const std::array<Math::Vector3d, 2>& qT1,
		double thicknessP,
		double thicknessQ,
		double timePrecisionEpsilon,
		double* t, double* r, double* s);

	/// Manage the specific case of detecting collisions between segments p and q which are parallel throughout
	/// the parametric time interval of interest [a, b].
	/// \param pT0 are the segment endpoints for the first segment at time t=0.
	/// \param pT1 are the segment endpoints for the first segment at time t=1.
	/// \param qT0 are the segment endpoints for the second segment at time t=0.
	/// \param qT1 are the segment endpoints for the second segment at time t=1.
	/// \param a parametric starting point of the interval of interest.
	/// \param b parametric ending point of the interval of interest.
	/// \param thicknessP radius of segment p.
	/// \param thicknessQ radius of segment q.
	/// \param timePrecisionEpsilon time nearness criteria for declaring a contact.
	/// \param t [out] parametric location of the collision along the time axes in the interval [0, 1]
	/// \param r [out] parametric location of the collision along p in the interval [0, 1]
	/// \param s [out] parametric location of the collision along q in the interval [0, 1]
	/// \param depth recursion depth.
	/// \return true if p and q collide in interval [a, b]
	bool collideSegmentSegmentParallelCase(
		const std::array<Math::Vector3d, 2>& pT0,
		const std::array<Math::Vector3d, 2>& pT1,
		const std::array<Math::Vector3d, 2>& qT0,
		const std::array<Math::Vector3d, 2>& qT1,
		double a, double b,
		double thicknessP, double thicknessQ,
		double timePrecisionEpsilon,
		double* t, double* r, double* s, int depth = 0);

	/// Manage the specific case of detecting collisions between segments p and q which are coplanar throughout
	/// the parametric time interval of interest [a, b].
	/// \param pT0 are the segment endpoints for the first segment at time t=0.
	/// \param pT1 are the segment endpoints for the first segment at time t=1.
	/// \param qT0 are the segment endpoints for the second segment at time t=0.
	/// \param qT1 are the segment endpoints for the second segment at time t=1.
	/// \param a parametric starting point of the interval of interest.
	/// \param b parametric ending point of the interval of interest.
	/// \param thicknessP radius of segment p.
	/// \param thicknessQ radius of segment q.
	/// \param timePrecisionEpsilon time nearness criteria for declaring a contact.
	/// \param t [out] parametric location of the collision along the time axes in the interval [0, 1]
	/// \param r [out] parametric location of the collision along p in the interval [0, 1]
	/// \param s [out] parametric location of the collision along q in the interval [0, 1]
	/// \param depth recursion depth.
	/// \return true if p and q collide in interval [a, b]
	bool collideSegmentSegmentCoplanarCase(
		const std::array<Math::Vector3d, 2>& pT0, /* Segment 1 at t=0 */
		const std::array<Math::Vector3d, 2>& pT1, /* Segment 1 at t=1 */
		const std::array<Math::Vector3d, 2>& qT0, /* Segment 2 at t=0 */
		const std::array<Math::Vector3d, 2>& qT1, /* Segment 2 at t=1 */
		double a, double b, /* Interval boundaries */
		double timePrecisionEpsilon,
		double thicknessP, double thicknessQ,
		double* t, double* r, double* s,
		int depth = 0);

	/// Manage the general case of detecting collisions between segments p and q over the parametric time
	/// interval [a, b] when no special spatial relationships can be observed that improve performance.
	/// \param state an encapsulation of the segment locations, movements, and detection parameters.
	/// \param a parametric starting point of the interval of interest.
	/// \param b parametric ending point of the interval of interest.
	/// \param t [out] parametric location of the collision along the time axes in the interval [0, 1]
	/// \param r [out] parametric location of the collision along p in the interval [0, 1]
	/// \param s [out] parametric location of the collision along q in the interval [0, 1]
	/// \param depth recursion depth.
	/// \return true if p and q collide in interval [a, b]
	bool collideSegmentSegmentGeneralCase(
		const SegmentSegmentCcdIntervalCheck& state,
		double a, double b, // Interval boundaries
		double* t, double* r, double* s,
		int depth = 0);

	/// Calculate the best unit normal we can find in the direction of pXq for one of the endpoints of q.
	/// Try multiple arrangements of the end points to reduce the artifacts when three of the vertices may
	/// be nearly collinear.
	/// \param p segment p
	/// \param q segment q
	/// \param epsilon when the norm of p x q is above epsilon, the cross product is assumed to be valid.
	/// return the normalized cross product of p x q
	Math::Vector3d calculatePXQ(const std::array<Math::Vector3d, 2>& p,
								const std::array<Math::Vector3d, 2>& q,
								double epsilon) const;

	/// Safely normalize segments t0 and t1 consistently with each other. Under the assumption that they
	/// both represent the same segment at two different time points. Ensure that for cases where the segment
	/// is too small at one or both time points (i.e. they essentially degenerate to a point) that we make
	/// an intelligent choice.
	/// \param t0 segment at time 0
	/// \param t1 segment at time 1
	/// \param epsilon threshold for valid normalization value.
	void normalizeSafely(Math::Vector3d* t0, Math::Vector3d* t1, double epsilon) const;

private:
	/// Utility routine to perform a series of checks to determine if a collision is likely within an interval. The
	/// checks seek to determine if two coplanar segments could have been out of contact at the start of an interval,
	/// and then moved through a contact and back away. Among the values checked are the normals because a change in
	/// normal direction of p X q indicates that p is now on the other side of q. Other checks are made for flipping
	/// segments, etc.
	/// \param rCurrent is the parametric location on segment p at the start of the current time interval
	/// \param rNext is the parametric location on segment p at the end of the current time interval
	/// \param sCurrent is the parametric location on segment q at the start of the current time interval
	/// \param sNext is the parametric location on segment q at the end of the current time interval
	/// \param numberSubpoints is the number of points in the current time subdivision
	/// \param nCurrent is the normal of p x q at the start of the current time interval
	/// \param nNext is the normal of p x q at the current time point
	/// \return true if the check indicates a collision may be possible for coplanar segments p and
	/// q at the current time interval.
	bool checkForCoplanarContactWithinInterval(double rCurrent, double rNext, double sCurrent, double sNext,
			int numberSubpoints, const Math::Vector3d& nCurrent, const Math::Vector3d& nNext) const;

	/// Minimum distance precision epsilon used in continuous collision detection.
	const double m_distanceEpsilon;

	/// Utility class for testing interval boundary collisions.
	Collision::SegmentSegmentCcdStaticContact m_staticTest;
};

}; // namespace Collision
}; // namespace SurgSim

#endif // SURGSIM_COLLISION_SEGMENTSEGMENTCCDMOVINGCONTACT_H

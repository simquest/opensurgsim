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

#ifndef SURGSIM_COLLISION_SEGMENTCCDSELFCONTACT_H
#define SURGSIM_COLLISION_SEGMENTCCDSELFCONTACT_H

//#include <memory>

#include "SurgSim/Collision/ShapeShapeContactCalculation.h"
#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Math/SegmentMeshShape.h"

namespace SurgSim
{
namespace Collision
{

class CollisionPair;

/// SegmentCcdSelfContact computes the self collisions among a SegmentMesh under motion at two
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
class SegmentCcdSelfContact : public ShapeShapeContactCalculation<Math::SegmentMeshShape, Math::SegmentMeshShape>
{
public:
	/// Constructor.
	SegmentCcdSelfContact();

	/// Calculate the contacts using the typed shapes directly
	/// \param segmentShape1 the first segment shape
	/// \param segmentPose1 the pose of the second segment
	/// \param segmentShape2 the second segment shape
	/// \param segmentPose2 the pose of the second segment
	/// \return a list of contacts between the shapes, if any
	std::list<std::shared_ptr<Contact>> calculateContact(
										 const Math::SegmentMeshShape& segmentShape1,
										 const Math::RigidTransform3d& segmentPose1,
										 const Math::SegmentMeshShape& segmentShape2,
										 const Math::RigidTransform3d& segmentPose2) const override;

	/// Set the minimum time precision allowed when deciding on the depth of recursion.
	/// \param precision the desired minimum time precision
	void setTimeMinPrecisionEpsilon(double precision);

	/// \return the minimum time precision allowed when deciding on the depth of recursion.
	double timeMinPrecisionEpsilon();

	/// Set the maximum time precision allowed when deciding on the depth of recursion.
	/// \param precision the desired maximum time precision
	void setTimeMaxPrecisionEpsilon(double precision);

	/// \return the maximum time precision allowed when deciding on the depth of recursion.
	double timeMaxPrecisionEpsilon();

	/// Set the maximum separation for which two points are considered the same.
	/// \param precision the desired maximum separation for which two points are considered the same.
	void setDistanceEpsilon(double precision);

	/// \return the maximum separation for which two points are considered the same.
	double distanceEpsilon();

	/// Function that returns the shapes between which this class performs collision detection.
	/// \return int std::pair containing the shape types.
	std::pair<int, int> getShapeTypes() override;

protected:
	/// Detect if two segments actually collide either at time t=0 (Fixed case) or within a movement phase.
	/// \param pt0Positions are the segment endpoints for the first segment at time t=0.
	/// \param pt1Positions are the segment endpoints for the first segment at time t=1.
	/// \param qt0Positions are the segment endpoints for the second segment at time t=0.
	/// \param qt1Positions are the segment endpoints for the second segment at time t=1.
	/// \param segmentRadius1 is the radius of the first segment.
	/// \param segmentRadius2 is the radius of the second segment.
	/// \param timePrecision is the minimum time delta. Recursion terminates at or below the timePrecision.
	/// \param r [out] parametric location of the collision point (if any) on segment 1.
	/// \param s [out] parametric location of the collision point (if any) on segment 2.
	/// \param t [out] parametric location of the time of any collision in the interval [0, 1].
	/// \param pToQDir [out] direction from p(s) to q(r)
	/// \param contactPtP [out] location of the contact point along segment 1.
	/// \param contactPtQ [out] location of the contact point along segment 2.
	/// \return true if a collision is detected between segment 1 and segment 2; false otherwise.
	bool detectCollision(
		const std::array<SurgSim::Math::Vector3d, 2>& pt0Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& pt1Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& qt0Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& qt1Positions,
		double segmentRadius1, double segmentRadius2,
		double timePrecision,
		double* r, double* s, double* t,
		SurgSim::Math::Vector3d* pToQDir,
		SurgSim::Math::Vector3d* contactPtP,
		SurgSim::Math::Vector3d* contactPtQ) const;

	/// Given a list of potentially intersecting AABB nodes, cull the list of any duplicates
	/// and return the uniques candidates as synchronized pairs.
	/// \param intersectionList list of potentially intersecting AABB node.
	/// \param segmentIdList [out] paired unique matches
	void getUniqueCandidates(
		const std::list<SurgSim::DataStructures::AabbTree::TreeNodePairType>& intersectionList,
		std::list<std::pair<size_t, size_t>>* segmentIdList) const;

	/// From the initial AABB tree collisions, there are some very simple filtering operations that we can
	/// do to eliminate a number of false positives. Most notably, we do not want to collide a single segment
	/// against itself, or against one of the segments with which it shares a vertex. These are trivial collisions
	/// and will always be present. Less obvious, we want to filter out segments with wild movement vectors.
	/// These segments cannot be appropriately processed and are likely to represent errors.
	/// \param segmentA the segment mesh at time t=0.
	/// \param segmentB the segment mesh at time t=1.
	/// \param segment1SegID the specific identifier of the candidate segment at time t=0.
	/// \param segment2SegID the specific identifier of the candidate segment at time t=1.
	/// \return true if this collision should be discarded, false i it should be processed further.
	bool preFilterCollision(
		const Math::SegmentMeshShape& segmentA,
		const Math::SegmentMeshShape& segmentB,
		size_t segment1SegID, size_t segment2SegID) const;

	/// Verify the a given point at times t0 and t1 have remained within a reasonable neighborhood. Large
	/// movements make it impossible to accurately determine collisions.
	/// \param pt0 vertex coordinates at time t0
	/// \param pt1 vertex coordinates at time t1
	/// \param threshold distance threshold
	/// \return true if every coordinate of the point has moved less than the threshold
	bool detectExcessMovement(const SurgSim::Math::Vector3d& pt0,
							  const SurgSim::Math::Vector3d& pt1,
							  double threshold) const;

	/// Search the list of contacts for a match to the current contact.
	/// \param segmentShape shape to be interrogated to see if the contacts match
	/// \param contacts the current list of detected contacts
	/// \param toi time of the contact under consideration.
	/// \param collisionType type of contact under consideration.
	/// \param segId1 segment 1 identifier.
	/// \param s1 parametric location of the contact point along segment 1.
	/// \param segId2 segment 2 identifier.
	/// \param s2 parametric location of the contact point along segment 2.
	/// \param timeEpsilon maximum allowed epsilon for time matches.
	/// \return true if the contacts match, return false otherwise.
	bool findSegSegContact(const Math::SegmentMeshShape& segmentShape,
						   const std::list<std::shared_ptr<Contact>>& contacts,
						   double toi, Collision::CollisionDetectionType collisionType,
						   size_t segId1, double s1, size_t segId2, double s2, double timeEpsilon) const;

	/// Check for the same location among two parametric location specifications.
	/// \param segmentShape shape to be interrogated to see if the contacts match
	/// \param segId1 segment 1 identifier.
	/// \param s1 parametric location of the contact point along segment 1.
	/// \param segId2 segment 2 identifier.
	/// \param s2 parametric location of the contact point along segment 2.
	/// \return true if the contacts match, return false otherwise.
	bool isSameSegContactPoint(const Math::SegmentMeshShape& segmentShape,
							   size_t segId1, double s1, size_t segId2, double s2) const;

	/// Calculate the maximum time interval that guarantees that all collisions can be detected given the
	/// derived motions of the segment end points.
	/// \param pt0Positions are the segment endpoints for the first segment at time t=0.
	/// \param pt1Positions are the segment endpoints for the first segment at time t=1.
	/// \param qt0Positions are the segment endpoints for the second segment at time t=0.
	/// \param qt1Positions are the segment endpoints for the second segment at time t=1.
	/// \param effectiveThickness nearness criteria for declaring a contact.
	/// \return the maximum time interval that will still allow for the detection of all
	/// contacts within effectveThickness. This value is bounded from below by the value of
	/// the member variable m_timeMinPrecisionEpsilon and from above by m_timeMaxPrecisionEpsilon.
	double maxTimePrecision(
		const std::array<SurgSim::Math::Vector3d, 2>& pt0Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& pt1Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& qt0Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& qt1Positions,
		double effectiveThickness) const;

private:
	/// Minimum distance precision epsilon used in continuous collision detection.
	double m_distanceEpsilon;

	/// Minimum time precision epsilon used in continuous collision detection.
	double m_timeMinPrecisionEpsilon;

	/// Maximum time precision epsilon used in continuous collision detection.
	double m_timeMaxPrecisionEpsilon;

	/// Maximum time precision epsilon used in continuous collision detection.
	const double m_maxMovementThreshold;

	/// Flag to determine if segment thickness is to be used in contact calculations
	const bool m_useSegmentThickness;

	/// Logger
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
};

}; // namespace Collision
}; // namespace SurgSim

#endif // SURGSIM_COLLISION_SEGMENTCCDSELFCONTACT_H

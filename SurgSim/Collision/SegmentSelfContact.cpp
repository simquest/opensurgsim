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

#include "SurgSim/Collision/SegmentSelfContact.h"

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Collision/SegmentSegmentCcdMovingContact.h"
#include "SurgSim/Collision/SegmentSegmentCcdStaticContact.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/Scalar.h"
#include "SurgSim/Math/SegmentMeshShape.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::MeshShape;

namespace SurgSim
{
namespace Collision
{

SegmentSelfContact::SegmentSelfContact():
	m_distanceEpsilon(1.0e-09),
	m_timeMinPrecisionEpsilon(1.0e-06),
	m_timeMaxPrecisionEpsilon(1.0e-06),
	m_maxMovementThreshold(0.1),
	m_useSegmentThickness(true),
	m_logger(Framework::Logger::getLogger("Collision/SegmentSelfContact"))
{
}

std::pair<int, int> SegmentSelfContact::getShapeTypes()
{
	return std::pair<int, int>(Math::SHAPE_TYPE_SEGMENTMESH, Math::SHAPE_TYPE_SEGMENTMESH);
}

void SegmentSelfContact::setTimeMinPrecisionEpsilon(double precision)
{
	SURGSIM_ASSERT(precision > 0.0) << "Cannot set a negative min/max time precision.";
	m_timeMinPrecisionEpsilon = precision;
}

double SegmentSelfContact::getTimeMinPrecisionEpsilon()
{
	return m_timeMinPrecisionEpsilon;
}

void SegmentSelfContact::setTimeMaxPrecisionEpsilon(double precision)
{
	SURGSIM_ASSERT(precision > 0.0) << "Cannot set a negative min/max time precision.";
	m_timeMaxPrecisionEpsilon = precision;
}

double SegmentSelfContact::getTimeMaxPrecisionEpsilon()
{
	return m_timeMaxPrecisionEpsilon;
}

void SegmentSelfContact::setDistanceEpsilon(double precision)
{
	m_distanceEpsilon = precision;
}

double SegmentSelfContact::distanceEpsilon()
{
	return m_distanceEpsilon;
}

std::list<std::shared_ptr<Contact>> SegmentSelfContact::calculateCcdContact(
									 const Math::SegmentMeshShape& segmentShape1AtTime0,
									 const Math::RigidTransform3d& segmentPose1AtTime0,
									 const Math::SegmentMeshShape& segmentShape1AtTime1,
									 const Math::RigidTransform3d& segmentPose1AtTime1,
									 const Math::SegmentMeshShape& segmentShape2AtTime0,
									 const Math::RigidTransform3d& segmentPose2AtTime0,
									 const Math::SegmentMeshShape& segmentShape2AtTime1,
									 const Math::RigidTransform3d& segmentPose2AtTime1) const
{

	const Math::SegmentMeshShape& segmentShape1 = segmentShape1AtTime0;
	const Math::RigidTransform3d& segmentPose1 = segmentPose1AtTime0;
	const Math::SegmentMeshShape& segmentShape2 = segmentShape1AtTime1;
	const Math::RigidTransform3d& segmentPose2 = segmentPose1AtTime1;

	SURGSIM_LOG_DEBUG(m_logger) << "============================ Inner Loop ================================";
	SURGSIM_ASSERT(segmentShape1.getNumEdges() == segmentShape2.getNumEdges()) <<
			"Segment CCD self collision detects that " <<
			"the segment at time t and time t + 1 have different numbers of edges.";

	std::list<std::shared_ptr<Contact>> contacts;

	// Intersect the AABB trees of the Segment Mesh at time 0 and time 1 to get a list of
	// potential intersecting segments.
	std::set<std::pair<size_t, size_t>> segmentIds;

	// TODO(wturner): We need to reinstitute the AABB tree to handle motion. When we do, the
	// following codecan be reenabled and the brute force mmethod below removed:
	//
	// Beginning of AABB
	//std::list<DataStructures::AabbTree::TreeNodePairType> intersectionList
	//	= segmentShape1.getAabbTree()->spatialJoin(*segmentShape2.getAabbTree());
	//getUniqueCandidates(intersectionList, &segmentIds);
	// End of AABB.
	for (size_t i = 0; i < segmentShape1.getNumEdges(); i++)
	{
		for (size_t j = i + 1; j < segmentShape1.getNumEdges(); j++)
		{
			segmentIds.emplace(i, j);
		}
	}


	for (const auto& idPair : segmentIds)
	{
		size_t id1 = idPair.first;
		size_t id2 = idPair.second;

		SURGSIM_ASSERT(id1 >= 0 && id1 < segmentShape1.getNumEdges()) << "Invalid segment detected in "
				<< "Segment CCD self collision. Colliding segment at time point 0 does not exist.";
		SURGSIM_ASSERT(id2 >= 0 && id2 < segmentShape2.getNumEdges()) << "Invalid segment detected in "
				<< "Segment CCD self collision. Colliding segment at time point 1 does not exist.";

		// Do a little filtering. We do not allow a segment to collide with itself or
		// with an immediate neighbor; and pragmatically, it seems reasonable to disregard any segments that show
		// too much movement between times. At best this means that we should be taking smaller time steps,
		// but it is probably more likely to reflect some other error such as an unstable solution.
		if (removeInvalidCollisions(segmentShape1, segmentShape2, id1, id2))
		{
			continue;
		}

		const auto& pt0Positions = segmentShape1.getEdgePositions(id1);
		const auto& pt1Positions = segmentShape2.getEdgePositions(id1);
		const auto& qt0Positions = segmentShape1.getEdgePositions(id2);
		const auto& qt1Positions = segmentShape2.getEdgePositions(id2);

		double segmentRadius1 = 0.0;
		double segmentRadius2 = 0.0;
		double effectiveThickness = m_distanceEpsilon;
		if (m_useSegmentThickness)
		{
			// TODO(wdturner-11/2015): We need to get thickness as a property. Until then use
			// the radius ...
			//
			// segmentRadius1 = segmentA->getEdge(id1).data.thickness;
			// segmentRadius2 = segmentB->getEdge(id2).data.thickness;
			segmentRadius1 = segmentShape1.getRadius();
			segmentRadius2 = segmentShape2.getRadius();
			effectiveThickness = segmentRadius1 + segmentRadius2;
		}

		//
		// Based on movement speed, calculate the maximum time interval that will maintain detection accuracy.
		//
		double timePrecision = maxTimePrecision(pt0Positions, pt1Positions, qt0Positions, qt1Positions,
												effectiveThickness);

		double pLen = (pt0Positions[1] - pt0Positions[0]).squaredNorm();
		double rParametricPrecision = m_distanceEpsilon;
		if (pLen > 0.0)
		{
			rParametricPrecision = std::min(0.5, m_distanceEpsilon / std::sqrt(pLen));
		}

		double qLen = (pt1Positions[1] - pt1Positions[0]).squaredNorm();
		double sParametricPrecision = m_distanceEpsilon;
		if (qLen > 0.0)
		{
			sParametricPrecision = std::min(0.5, m_distanceEpsilon / std::sqrt(qLen));
		}

		// Perform the actual collision detection and create the contact.
		double r;	// Parametric location of collision on segment p
		double s;	// Parametric location of collision on segment q
		double t;	// Time of collision (if any)
		Math::Vector3d pToQDir;
		Math::Vector3d segmentPContact;
		Math::Vector3d segmentQContact;
		if (detectCollision(pt0Positions, pt1Positions, qt0Positions, qt1Positions,
							segmentRadius1, segmentRadius2, timePrecision,
							&r, &s, &t, &pToQDir, &segmentPContact, &segmentQContact))
		{
			// The segments collide within tolerance, but if the collision is really close to an endpoint
			// then move it to the start of the segment to aid in removing duplicates.
			r = Math::clamp(r, 0.0, 1.0, rParametricPrecision);
			s = Math::clamp(s, 0.0, 1.0, sParametricPrecision);

			// When a segment extremity collides, its collision can be detected twice,
			// as this point is shared between 2 segments! Here, we choose *one* of them to add!
			// Be sure to check both directions.
			if (!findSegSegContact(segmentShape1, contacts, t,
								   COLLISION_DETECTION_TYPE_CONTINUOUS,
								   id1,  r, id2, s, timePrecision) &&
				!findSegSegContact(segmentShape1, contacts, t,
								   COLLISION_DETECTION_TYPE_CONTINUOUS,
								   id2,  s, id1, r, timePrecision))
			{
				// Encode the segment specific intersection points for later recall. Here we encode each
				// side of the contact point specific to each segment.
				std::pair<Location, Location> penetrationPoints;
				Math::Vector2d parametricCoordinateP(1.0 - r, r);
				penetrationPoints.first.elementMeshLocalCoordinate.setValue(
					DataStructures::IndexedLocalCoordinate(id1, parametricCoordinateP));
				Math::Vector2d parametricCoordinateQ(1.0 - s, s);
				penetrationPoints.second.elementMeshLocalCoordinate.setValue(
					DataStructures::IndexedLocalCoordinate(id2, parametricCoordinateQ));
				penetrationPoints.first.rigidLocalPosition.setValue(
					segmentPose1.inverse() * segmentPContact);
				penetrationPoints.second.rigidLocalPosition.setValue(
					segmentPose2.inverse() * segmentQContact);

				// Calculate the normal and the contact point. The contact point is a point along the normal
				// connecting the segments. If we are using the segment thickness, the point should be located
				// a segment radius distance from each individual contact point. Otherwise, it should be within
				// m_distanceEpsilon/2. For accuracy, it is calculated from both starting points and then averaged.
				double effectiveRadiusP = m_useSegmentThickness ? segmentRadius1 : m_distanceEpsilon / 2.0;
				double effectiveRadiusQ = m_useSegmentThickness ? segmentRadius2 : m_distanceEpsilon / 2.0;

				auto normal = pToQDir.normalized();
				Math::Vector3d contactP = segmentPContact + (effectiveRadiusP * normal);
				Math::Vector3d contactQ = segmentQContact - (effectiveRadiusQ * normal);
				auto contactPoint = 0.5 * (contactP + contactQ);
				auto depth = ((contactP - contactQ).dot(normal) > 0.0) ?
							 (contactP - contactQ).norm() : -(contactP - contactQ).norm();
				SURGSIM_LOG_DEBUG(m_logger) << "time:\t" << t << "\tid1:\t" << id1 << "\tid2:\t" << id2 <<
											"\tr:\t" << r << "\ts:\t" << s << "\tNormal:\t" <<
											pToQDir.norm() << "\tDepth:\t" << depth;
				contacts.emplace_back(std::make_shared<Contact>(
										  CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, depth, t,
										  contactPoint, -normal, penetrationPoints));
			}
		}
		else
		{
//			SURGSIM_LOG_DEBUG(m_logger) <<
//										"AABB tree detected false positive between segments " << id1 << " and " << id2;
		}
	}
	return contacts;
}

bool SegmentSelfContact::detectCollision(
	const std::array<Math::Vector3d, 2>& pt0Positions,
	const std::array<Math::Vector3d, 2>& pt1Positions,
	const std::array<Math::Vector3d, 2>& qt0Positions,
	const std::array<Math::Vector3d, 2>& qt1Positions,
	double segmentRadius1, double segmentRadius2, double timePrecision,
	double* r, double* s, double* t,
	Math::Vector3d* pToQDir,
	Math::Vector3d* contactPtP,
	Math::Vector3d* contactPtQ) const
{
	//
	// First check for intersection at the start of the interval. If this is true, we can just report on the
	// contact at t=0 and return. Note that the original code compared totalThickness == 0.0. This is slightly
	// different in that it assumes that totalThickness cannot be less than m_distanceEpsilon since
	// m_distanceEpsilon represents the 0 segment thickness case.
	//
	// Important safety note. These calls to compute the static segment collision originally had two flavors: one
	// for segments with thickness and one for segments with no thickness (which used an attributed thickness of
	// m_distanceEpsilon). I dug into this and the only difference in the two was in the handling of potential
	// intersections beyond the segment end point. For segments with thickness, the intersection point was
	// clamped to the end of the segment but was deemed real if the modified endpoint was within an acceptable
	// distance of the true intersection point as determined by the segment radius. For the zero radius case,
	// this margin at the segment ends was not observed. While this implementation maintains the two different
	// APIs, it does not maintain the algorithmic differences. The duplicate collision removal as implemented
	// should resolve any places within a string of segments where extra collisions are detected and the only
	// observable change should be at unattached segment boundaries where it is arguable as to which method is
	// more correct. Certainly, it will be easier to selectively impose this condition should it prove desirable,
	// than it is to maintain two nearly identical methods as was done previously.
	//
	double totalThickness = segmentRadius1 + segmentRadius2;
	bool collidingAtT0;
	static SegmentSegmentCcdStaticContact staticContact;
	if (totalThickness <= m_distanceEpsilon)
	{
		collidingAtT0 = staticContact.collideStaticSegmentSegment(pt0Positions, qt0Positions,
						m_distanceEpsilon, r, s);
	}
	else
	{
		collidingAtT0 = staticContact.collideStaticSegmentSegment(pt0Positions, qt0Positions,
						segmentRadius1, segmentRadius2, r, s);
	}

	if (collidingAtT0)
	{
		*t = 0;
		*contactPtP = Math::interpolate(pt0Positions[0], pt0Positions[1], *r);
		*contactPtQ = Math::interpolate(qt0Positions[0], qt0Positions[1], *s);
		*pToQDir = *contactPtQ - *contactPtP;
		return true;
	}

	//
	// Finally, detect if the segments collide throughout the period of movement.
	//
	bool collisionDetected;

	static SegmentSegmentCcdMovingContact movingContact;
	if (totalThickness < m_distanceEpsilon)
	{
		collisionDetected = movingContact.collideMovingSegmentSegment(pt0Positions, pt1Positions,
							qt0Positions, qt1Positions,
							m_distanceEpsilon, timePrecision,
							t, r, s, pToQDir);
	}
	else
	{
		collisionDetected = movingContact.collideMovingSegmentSegment(pt0Positions, pt1Positions,
							qt0Positions, qt1Positions,
							segmentRadius1, segmentRadius2, timePrecision, t, r, s, pToQDir);
	}

	//
	// If a collision is detected, use interpolation to determine the point in time and space.
	//
	if (collisionDetected)
	{
		Math::Vector3d p0Contact = Math::interpolate(pt0Positions[0], pt0Positions[1], *r);
		Math::Vector3d p1Contact = Math::interpolate(pt1Positions[0], pt1Positions[1], *r);
		Math::Vector3d q0Contact = Math::interpolate(qt0Positions[0], qt0Positions[1], *s);
		Math::Vector3d q1Contact = Math::interpolate(qt1Positions[0], qt1Positions[1], *s);

		*contactPtP = Math::interpolate(p0Contact, p1Contact, *t);
		*contactPtQ = Math::interpolate(q0Contact, q1Contact, *t);

		*pToQDir = *contactPtQ - *contactPtP;
	}

	return collisionDetected;
}

void SegmentSelfContact::getUniqueCandidates(
	const std::list<SurgSim::DataStructures::AabbTree::TreeNodePairType>& intersectionList,
	std::set<std::pair<size_t, size_t>>* segmentIds) const
{
	for (const auto& intersection : intersectionList)
	{
		std::shared_ptr<DataStructures::AabbTreeNode> nodeA = intersection.first;
		std::shared_ptr<DataStructures::AabbTreeNode> nodeB = intersection.second;
		std::list<size_t> localIdListA;
		std::list<size_t> localIdListB;

		nodeA->getIntersections(nodeB->getAabb(), &localIdListA);
		nodeB->getIntersections(nodeA->getAabb(), &localIdListB);
		for (const auto& idA : localIdListA)
		{
			for (const auto& idB : localIdListB)
			{
				// Segments are the same
				if (idA == idB)
				{
					continue;
				}

				// Segment pair has already been added
				auto  testValue = (idA < idB) ? std::pair<size_t, size_t>(idA, idB) :
								  std::pair<size_t, size_t>(idB, idA);
				segmentIds->insert(testValue);
			}
		}
	}
}

bool SegmentSelfContact::removeInvalidCollisions(
	const Math::SegmentMeshShape& segmentT0,
	const Math::SegmentMeshShape& segmentT1,
	size_t id1, size_t id2) const
{
	auto& verticesA = segmentT0.getEdge(id1).verticesId;
	auto& verticesB = segmentT1.getEdge(id2).verticesId;
	if ((verticesA[0] == verticesB[0]) ||
		(verticesA[0] == verticesB[1]) ||
		(verticesA[1] == verticesB[0]) ||
		(verticesA[1] == verticesB[1]))
	{
		SURGSIM_LOG_DEBUG(m_logger)
				<< "Locality: Edge " << id1 << " and "
				<< id2 << " share a common vertex.";
		return true;
	}

	const auto& pt0Positions = segmentT0.getEdgePositions(id1);
	const auto& pt1Positions = segmentT1.getEdgePositions(id1);
	const auto& qt0Positions = segmentT0.getEdgePositions(id2);
	const auto& qt1Positions = segmentT1.getEdgePositions(id2);

	auto p1t0 = pt0Positions[0];
	auto p2t0 = pt0Positions[1];
	auto p1t1 = pt1Positions[0];
	auto p2t1 = pt1Positions[1];

	auto q1t0 = qt0Positions[0];
	auto q2t0 = qt0Positions[1];
	auto q1t1 = qt1Positions[0];
	auto q2t1 = qt1Positions[1];

	if (detectExcessMovement(p1t0, p1t1, m_maxMovementThreshold) ||
		detectExcessMovement(p2t0, p2t1, m_maxMovementThreshold) ||
		detectExcessMovement(q1t0, q1t1, m_maxMovementThreshold) ||
		detectExcessMovement(q2t0, q2t1, m_maxMovementThreshold))
	{
		SURGSIM_LOG_WARNING(m_logger)
				<< "Excessive movement detected during contact evaluation";
		return true;
	}
	return false;
}

bool SegmentSelfContact::detectExcessMovement(const Math::Vector3d& pt0,
		const Math::Vector3d& pt1,
		double threshold) const
{
	return (std::abs(pt0[0] - pt1[0]) > threshold) ||
		   (std::abs(pt0[1] - pt1[1]) > threshold) ||
		   (std::abs(pt0[2] - pt1[2]) > threshold);

}

bool SegmentSelfContact::findSegSegContact(const Math::SegmentMeshShape& segmentShape,
		const std::list<std::shared_ptr<Contact>>& contacts,
		double t, Collision::CollisionDetectionType collisionType, size_t segId1, double s1,
		size_t segId2, double s2, double timeEpsilon) const
{
	for (const auto& contact : contacts)
	{
		auto existingSegId1 = contact->penetrationPoints.first.elementMeshLocalCoordinate.getValue().index;
		auto existingSegId2 = contact->penetrationPoints.second.elementMeshLocalCoordinate.getValue().index;
		auto existingS1 = contact->penetrationPoints.first.elementMeshLocalCoordinate.getValue().coordinate[1];
		auto existingS2 = contact->penetrationPoints.second.elementMeshLocalCoordinate.getValue().coordinate[1];

		// Check for same object type and same time. The test for same objectID was
		// removed as it did not appear to be used (all find segment calls set that
		// variable to -1) and because it is not available here.
		if (contact->type == collisionType &&
			std::abs(contact->time - t) < timeEpsilon)
		{
			// Check that it is the same contact point on both segments
			if (isSameSegContactPoint(segmentShape, segId1, s1, existingSegId1, existingS1))
			{
				// Colliding against the same segment (no need to check the abscissa)
				// Then check for end point cases, which can have different segIDs
				if ((existingSegId2 == segId2) ||
					isSameSegContactPoint(segmentShape, segId2, s2, existingSegId2, existingS2))
				{
					return true;
				}

			}
		}
	}
	return false;
}

bool SegmentSelfContact::isSameSegContactPoint(const Math::SegmentMeshShape& segmentShape,
		size_t segId1, double s1, size_t segId2, double s2) const
{

	auto& verticesA = segmentShape.getEdge(segId1).verticesId;
	auto& verticesB = segmentShape.getEdge(segId2).verticesId;

	if (segId1 == segId2 &&						// Same segment?
		std::fabs(s1 - s2) < m_distanceEpsilon)			// Same abscissa?
	{
		return true;
	}

	// Check for segment extremities, which may be shared with another segment. Unlike the previous version,
	// which used ordering of the segments to determine "sharedness" this version cannot rely on ordering. Instead,
	// we need to determine the vertex ids and compare them. Assuming a vertex may exhibit either a branching or
	// a linear structure, this gives us 4 cases (1a, 1b, 2a, 2b). 1 and 2 represent vertices where the segId
	// is at the beginning of the segment (s==0), or at the end of the segment (s==1); while a and b represent
	// the equivalent position on the test vertex.

	// Test for segId at the start of the segment (1a and 1b)
	if (s1 < m_distanceEpsilon)
	{
		return ((s2 < m_distanceEpsilon) && verticesA[0] == verticesB[0]) ||
			   ((s2 > (1.0 - m_distanceEpsilon) && verticesA[0] == verticesB[1]));
	}

	// Test for segId at the end of the segment (2a and 2b)
	if (s1 > (1.0 - m_distanceEpsilon))
	{
		return ((s2 < m_distanceEpsilon) && verticesA[1] == verticesB[0]) ||
			   ((s2 > (1.0 - m_distanceEpsilon) && verticesA[1] == verticesB[1]));
	}

	return false;
}

double SegmentSelfContact::maxTimePrecision(
	const std::array<Math::Vector3d, 2>& pt0Positions,
	const std::array<Math::Vector3d, 2>& pt1Positions,
	const std::array<Math::Vector3d, 2>& qt0Positions,
	const std::array<Math::Vector3d, 2>& qt1Positions,
	double effectiveThickness) const
{
	SURGSIM_ASSERT(effectiveThickness > 0.0) << "Attempting CCD with segment thickness <= 0.";

	// Find the displacement of the endpoints during the time period
	auto p0Displacement = pt1Positions[0] - pt0Positions[0];
	auto p1Displacement = pt1Positions[1] - pt0Positions[1];
	auto q1Displacement = qt1Positions[0] - qt0Positions[0];
	auto q2Displacement = qt1Positions[1] - qt0Positions[1];

	// Get the relative displacement between the segments, so we know the relative movement
	double p1q1RelativeDisplacementNorm2 = (p0Displacement - q1Displacement).squaredNorm();
	double p1q2RelativeDisplacementNorm2 = (p0Displacement - q2Displacement).squaredNorm();
	double p2q1RelativeDisplacementNorm2 = (p1Displacement - q1Displacement).squaredNorm();
	double p2q2RelativeDisplacementNorm2 = (p1Displacement - q2Displacement).squaredNorm();

	// Now calculate the maximum relative displacement
	double maxP1RelativeDisplacementNorm2 = std::max(p1q1RelativeDisplacementNorm2, p1q2RelativeDisplacementNorm2);
	double maxP2RelativeDisplacementNorm2 = std::max(p2q1RelativeDisplacementNorm2, p2q2RelativeDisplacementNorm2);
	double maxRelativeDisplacement =
		std::sqrt(std::max(maxP1RelativeDisplacementNorm2, maxP2RelativeDisplacementNorm2));

	// The time precision should be set low enough so that we don't miss any segment collision.
	double timePrecision = effectiveThickness / maxRelativeDisplacement;

	if (timePrecision < m_timeMinPrecisionEpsilon)
	{
		SURGSIM_LOG_ONCE(m_logger, WARNING) <<
											"Minimum time precision(" << m_timeMinPrecisionEpsilon <<
											") needs to be smaller(" << timePrecision <<
											") to guarantee no missed self - collisions!";
		timePrecision = m_timeMinPrecisionEpsilon;
	}
	timePrecision = std::min(timePrecision, m_timeMaxPrecisionEpsilon);

	return timePrecision;
}

}; // namespace Collision
}; // namespace SurgSim

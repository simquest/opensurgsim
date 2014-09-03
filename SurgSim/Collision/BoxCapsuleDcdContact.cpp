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

#include <Eigen/Core>

#include "SurgSim/Collision/BoxCapsuleDcdContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/CapsuleShape.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::BoxShape;
using SurgSim::Math::CapsuleShape;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::doesIntersectBoxCapsule;
using SurgSim::Math::distancePointSegment;
using SurgSim::Math::intersectionsSegmentBox;
using SurgSim::Math::Geometry::DistanceEpsilon;

namespace {

typedef Eigen::AlignedBox<double, 3>::CornerType CornerType;

const std::array<std::pair<CornerType, CornerType>, 12> edges = { 
	std::make_pair(CornerType::BottomLeftFloor, CornerType::TopLeftFloor),
	std::make_pair(CornerType::BottomRightFloor, CornerType::TopRightFloor),
	std::make_pair(CornerType::BottomLeftCeil, CornerType::TopLeftCeil),
	std::make_pair(CornerType::BottomRightCeil, CornerType::TopRightCeil),

	std::make_pair(CornerType::BottomLeftFloor, CornerType::BottomRightFloor),
	std::make_pair(CornerType::BottomLeftCeil, CornerType::BottomRightCeil),
	std::make_pair(CornerType::TopLeftFloor, CornerType::TopRightFloor),
	std::make_pair(CornerType::TopLeftCeil, CornerType::TopRightCeil),

	std::make_pair(CornerType::BottomLeftFloor, CornerType::BottomLeftCeil),
	std::make_pair(CornerType::BottomRightFloor, CornerType::BottomRightCeil),
	std::make_pair(CornerType::TopLeftFloor, CornerType::TopLeftCeil),
	std::make_pair(CornerType::TopRightFloor, CornerType::TopRightCeil)
};
};

namespace SurgSim
{
namespace Collision
{

BoxCapsuleDcdContact::BoxCapsuleDcdContact()
{
}

std::pair<int,int> BoxCapsuleDcdContact::getShapeTypes()
{
	return std::pair<int,int>(SurgSim::Math::SHAPE_TYPE_BOX, SurgSim::Math::SHAPE_TYPE_CAPSULE);
}

void BoxCapsuleDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<Representation> boxRepresentation = pair->getFirst();
	std::shared_ptr<Representation> capsuleRepresentation = pair->getSecond();

	std::shared_ptr<CapsuleShape> capsuleShape =
		std::static_pointer_cast<CapsuleShape>(capsuleRepresentation->getShape());
	std::shared_ptr<BoxShape> boxShape = std::static_pointer_cast<BoxShape>(boxRepresentation->getShape());

	RigidTransform3d boxPose = boxRepresentation->getPose();
	RigidTransform3d capsuleToBoxTransform = boxPose.inverse() * capsuleRepresentation->getPose();
	Vector3d capsuleBottom = capsuleToBoxTransform * capsuleShape->bottomCenter();
	Vector3d capsuleTop = capsuleToBoxTransform * capsuleShape->topCenter();
	double capsuleRadius = capsuleShape->getRadius();

	Vector3d boxRadii = boxShape->getSize() / 2.0;
	Eigen::AlignedBox<double, 3> box(-boxRadii, boxRadii);

	if (doesIntersectBoxCapsule(capsuleBottom, capsuleTop, capsuleRadius, box))
	{
		Vector3d normal, segmentPoint, deepestBoxPoint, deepestCapsulePoint;
		distancePointSegment(Vector3d::Zero().eval(), capsuleBottom, capsuleTop, &segmentPoint);
		if (!segmentPoint.isZero(DistanceEpsilon))
		{
			// The capsule's segment does not pass through the box center.
			if (box.contains(segmentPoint))
			{
				// The capsule's segment passes through the box.
				Vector3d::Index closestFace;
				(boxRadii - segmentPoint.cwiseAbs()).minCoeff(&closestFace);
				normal.setZero();
				normal[closestFace] = -segmentPoint[closestFace];
				normal.normalize();
				deepestBoxPoint = boxRadii.array() * (1 - 2 * (segmentPoint.array() < 0).cast<double>());
				deepestCapsulePoint = segmentPoint - capsuleRadius * segmentPoint.normalized();
			}
			else
			{
				// The closest point on the capsule's segment to the center of the box is outside the box.
				deepestBoxPoint = segmentPoint.array().min(box.max().array()).max(box.min().array());
				normal = deepestBoxPoint - segmentPoint;
				if (normal.norm() > capsuleRadius)
				{
					// The closest point to the box center is too far away.
					// Find the closest point to all 12 box edges.
					double minDistance = 2.0 * capsuleRadius;
					for (auto edge : edges)
					{
						Vector3d tempSegmentPoint;
						Vector3d tempBoxPoint;
						double tempDistance = SurgSim::Math::distanceSegmentSegment(capsuleBottom, capsuleTop,
							box.corner(edge.first), box.corner(edge.second),
							&tempSegmentPoint, &tempBoxPoint);
						if (tempDistance < minDistance)
						{
							minDistance = tempDistance;
							segmentPoint = tempSegmentPoint;
							deepestBoxPoint = tempBoxPoint;
						}
					}
					normal = deepestBoxPoint - segmentPoint;
					if (normal.norm() > capsuleRadius)
					{
						// The closest point to any edge is too far away.
						// Check the endpoints.
						segmentPoint = capsuleTop;
						deepestBoxPoint = segmentPoint.array().min(box.max().array()).max(box.min().array());
						normal = deepestBoxPoint - segmentPoint;
						if (normal.norm() > capsuleRadius)
						{
							segmentPoint = capsuleBottom;
							deepestBoxPoint = segmentPoint.array().min(box.max().array()).max(box.min().array());
							normal = deepestBoxPoint - segmentPoint;
						}
					}
				}
				normal.normalize();
				deepestCapsulePoint = segmentPoint + capsuleRadius * normal;
			}
		}
		else
		{
			// The capsule's segment passes through the box center.
			if (capsuleTop.isZero(DistanceEpsilon) && capsuleBottom.isZero(DistanceEpsilon))
			{
				// The capsule's segment has no length and is located at the box center.
				Vector3d::Index closestFace;
				boxRadii.minCoeff(&closestFace);
				normal.setZero();
				normal[closestFace] = -boxRadii[closestFace];
				normal.normalize();
			}
			else
			{
				// The capsule's segment has a length, pick the closest endpoint to the box center.
				if (capsuleTop.squaredNorm() < capsuleBottom.squaredNorm())
				{
					segmentPoint = capsuleTop;
					normal = -capsuleBottom.normalized();
				}
				else
				{
					segmentPoint = capsuleBottom;
					normal = -capsuleTop.normalized();
				}
			}
			deepestBoxPoint = boxRadii.array() * (1 - 2 * (normal.array() > 0).cast<double>());
			deepestCapsulePoint = segmentPoint + capsuleRadius * normal;
		}

		double distance = (deepestCapsulePoint - deepestBoxPoint).dot(normal);
		std::pair<Location, Location> penetrationPoints;
		penetrationPoints.first.rigidLocalPosition.setValue(deepestBoxPoint);
		penetrationPoints.second.rigidLocalPosition.setValue(
			capsuleRepresentation->getPose().inverse() * boxPose * deepestCapsulePoint);
		pair->addContact(distance, boxPose.linear() * normal, penetrationPoints);
	}
}

}; // namespace Collision
}; // namespace SurgSim

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

using SurgSim::Math::BoxShape;
using SurgSim::Math::CapsuleShape;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::distancePointSegment;
using SurgSim::Math::intersectionsSegmentBox;
using SurgSim::Math::Geometry::DistanceEpsilon;

namespace SurgSim
{
namespace Collision
{

Vector3d clamp(const Vector3d& vector, const Vector3d& min, const Vector3d& max)
{
	Vector3d clampedVector = vector;
	for (int i = 0; i < 3; i++)
	{
		if (vector[i] > max[i])
		{
			clampedVector[i] = max[i];
		}
		else if (vector[i] < min[i])
		{
			clampedVector[i] = min[i];
		}
	}
	return clampedVector;
}

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
	Vector3d capsuleBottom = capsuleToBoxTransform * capsuleShape->bottomCentre();
	Vector3d capsuleTop = capsuleToBoxTransform * capsuleShape->topCentre();
	double capsuleRadius = capsuleShape->getRadius();

	Vector3d boxRadii = boxShape->getSize() / 2.0;
	Eigen::AlignedBox<double, 3> box(-boxRadii, boxRadii);

	Vector3d dilatedBoxRadii = boxRadii + Vector3d::Constant(capsuleRadius);
	Eigen::AlignedBox<double, 3> dilatedBox(-dilatedBoxRadii, dilatedBoxRadii);

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
	for(auto candidate=candidates.cbegin(); candidate!=candidates.cend(); ++candidate)
	{
		// Collisions between a capulse and a box are the same as a segment and a dilated
		// box with rounded corners. If the intersection occurs outside the original box
		// in two dimensions (collision with an edge of the dilated box) or three
		// dimensions (collision with the corner of the dilated box) dimensions, we need
		// to check if it is inside these rounded corners.
		dimensionsOutsideBox = (candidate->array() > box.max().array()).count();
		dimensionsOutsideBox += (candidate->array() < box.min().array()).count();
		if ((dimensionsOutsideBox >= 2))
		{
			clampedPosition = clamp(*candidate, box.min(), box.max());
			if (distancePointSegment(clampedPosition, capsuleBottom, capsuleTop, &segmentPoint) > capsuleRadius)
			{
				// Doesn't intersect, try the next candidate.
				continue;
			}
		}
		doesIntersect = true;
		break;
	}

	if (doesIntersect)
	{
		distancePointSegment(Vector3d::Zero().eval(), capsuleBottom, capsuleTop, &segmentPoint);
		Vector3d normal;
		if (segmentPoint.isZero(DistanceEpsilon))
		{
			Vector3d capsuleCenter = (capsuleTop + capsuleBottom) / 2.0;
			if (capsuleCenter.isZero(DistanceEpsilon))
			{
				if (capsuleTop.isZero(DistanceEpsilon))
				{
					Vector3d::Index closestFace;
					dilatedBoxRadii.minCoeff(&closestFace);
					normal = Vector3d::Unit(closestFace).array() * dilatedBoxRadii.array();
				}
				else
				{
					normal =  capsuleTop.normalized();
				}
			}
			else
			{
				normal = capsuleCenter.normalized();
			}
		}
		else
		{
			normal = segmentPoint.normalized();
		}

		Vector3d deepestPoint = Vector3d::Zero();
		for (int i=0; i<3; i++)
		{
			if (normal[i] > DistanceEpsilon)
			{
				deepestPoint[i] = dilatedBoxRadii[i];
			}
			else if (normal[i] < -DistanceEpsilon)
			{
				deepestPoint[i] = -dilatedBoxRadii[i];
			}
		}
		double distance = (deepestPoint - segmentPoint).dot(normal);

		std::pair<Location, Location> penetrationPoints;
		penetrationPoints.first.globalPosition.setValue(boxPose * clamp(deepestPoint, box.min(), box.max()));
		penetrationPoints.second.globalPosition.setValue(boxPose * (segmentPoint - capsuleRadius*normal));
		pair->addContact(distance, boxPose.linear() * (-normal), penetrationPoints);
	}
}

}; // namespace Collision
}; // namespace SurgSim

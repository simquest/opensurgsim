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

#include <SurgSim/Collision/BoxCapsuleDcdContact.h>

#include <Eigen/Core>

#include <SurgSim/Collision/CollisionPair.h>
#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/BoxShape.h>
#include <SurgSim/Math/CapsuleShape.h>

using SurgSim::Math::BoxShape;
using SurgSim::Math::CapsuleShape;
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
	for (int i=0; i<3; i++)
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

Vector3d directionToClosestEdge(const Vector3d& pt, const Eigen::AlignedBox<double, 3>& box)
{
	Vector3d direction;
	for (int i=0; i<3; i++)
	{
		if (pt[i] > 0)
		{
			direction[i] = box.max()[i] - pt[i];
		}
		else
		{
			direction[i] = box.min()[i] - pt[i];
		}
	}
	Vector3d::Index index;
	direction.cwiseAbs().maxCoeff(&index);
	direction[index] = 0.0;
	return direction;
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
	std::shared_ptr<CollisionRepresentation> boxRepresentation = pair->getFirst();
	std::shared_ptr<CollisionRepresentation> capsuleRepresentation = pair->getSecond();

	std::shared_ptr<CapsuleShape> capsuleShape =
		std::static_pointer_cast<CapsuleShape>(capsuleRepresentation->getShape());
	std::shared_ptr<BoxShape> boxShape = std::static_pointer_cast<BoxShape>(boxRepresentation->getShape());

	SurgSim::Math::RigidTransform3d capsuleToBoxTransform;
	capsuleToBoxTransform = boxRepresentation->getPose().inverse() * capsuleRepresentation->getPose();
	Vector3d capsuleBottom = capsuleToBoxTransform * capsuleShape->bottomCentre();
	Vector3d capsuleTop = capsuleToBoxTransform * capsuleShape->topCentre();
	double capsuleRadius = capsuleShape->getRadius();

	Vector3d boxRadii = boxShape->getSize() / 2.0;
	Eigen::AlignedBox<double, 3> box(-boxRadii, boxRadii);

	Vector3d dilatedBoxRadii = boxRadii + Vector3d::Constant(capsuleRadius);
	Eigen::AlignedBox<double, 3> dilatedBox(-dilatedBoxRadii, dilatedBoxRadii);

	std::vector<Vector3d> intersections;
	intersectionsSegmentBox(capsuleBottom, capsuleTop, dilatedBox, &intersections);

	if (dilatedBox.contains(capsuleBottom))
	{
		intersections.push_back(capsuleBottom);
	}

	if (dilatedBox.contains(capsuleTop))
	{
		intersections.push_back(capsuleTop);
	}

	Vector3d penetration, intersectionGlobal, clampedIntersection, unusedResult;
	double distance;
	int dimensionsOutsideBox;
	for(auto intersection=intersections.cbegin(); intersection!=intersections.cend(); ++intersection)
	{
		dimensionsOutsideBox = (intersection->array() > box.max().array()).count();
		dimensionsOutsideBox += (intersection->array() < box.min().array()).count();
		clampedIntersection = clamp(*intersection, box.min(), box.max());

		// Collisions between a capulse and a box are the same as a segment and a dilated
		// box with rounded corners. If the intersection occurs outside the original box
		// in two dimensions (collision with an edge of the dilated box) or three
		// dimensions (collision with the corner of the dilated box) dimensions, we need
		// to check if it is inside these rounded corners.
		if ((dimensionsOutsideBox < 2) ||
		   (distancePointSegment(clampedIntersection, capsuleBottom, capsuleTop, &unusedResult) < capsuleRadius))
		{
			penetration = directionToClosestEdge(*intersection, dilatedBox);

			// If intersection is an internal point (ie the capsule top and bottom),
			// move it to the closest face instead
			if ((penetration.cwiseAbs().array() < DistanceEpsilon).count() == 1)
			{
				Vector3d::Index index;
				penetration.cwiseAbs().minCoeff(&index);
				penetration[index] = 0.0;
			}

			penetration = boxRepresentation->getPose().linear() * penetration;
			intersectionGlobal = boxRepresentation->getPose() * (*intersection);

			std::pair<Location, Location> penetrationPoints;
			penetrationPoints.first.globalPosition.setValue(intersectionGlobal);
			penetrationPoints.second.globalPosition.setValue(intersectionGlobal + penetration);

			distance = penetration.norm();
			pair->addContact(distance, penetration / distance, penetrationPoints);
		}
	}
}

}; // namespace Collision
}; // namespace SurgSim

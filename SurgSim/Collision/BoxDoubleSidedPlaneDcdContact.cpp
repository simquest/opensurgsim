// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/BoxDoubleSidedPlaneDcdContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::BoxShape;
using SurgSim::Math::DoubleSidedPlaneShape;
using SurgSim::Math::Geometry::DistanceEpsilon;
using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Collision
{
std::pair<int, int> BoxDoubleSidedPlaneDcdContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_BOX, SurgSim::Math::SHAPE_TYPE_DOUBLESIDEDPLANE);
}

std::list<std::shared_ptr<Contact>> BoxDoubleSidedPlaneDcdContact::calculateContact(
									 const SurgSim::Math::BoxShape& boxShape,
									 const SurgSim::Math::RigidTransform3d& boxPose,
									 const SurgSim::Math::DoubleSidedPlaneShape& planeShape,
									 const SurgSim::Math::RigidTransform3d& planePose) const
{
	std::list<std::shared_ptr<Contact>> contacts;

	// Transform the plane normal to box co-ordinate system.
	SurgSim::Math::RigidTransform3d planeLocalToBoxLocal = boxPose.inverse() * planePose;
	Vector3d planeNormal = planeLocalToBoxLocal.linear() * planeShape.getNormal();
	Vector3d planeNormalScaled = planeShape.getNormal() * -planeShape.getD();
	Vector3d planePoint = planeLocalToBoxLocal * planeNormalScaled;
	double planeD = -planeNormal.dot(planePoint);

	// Loop through the box vertices (boxVertex) and calculate "d = (planeNormal.dot(boxVertex) + planeD)".
	// Keep track of max and min of 'd'.
	// Collision check overview:
	// - If 'd' values contain both positive and negative values, there is an intersection.
	// ---- Lower of the abs(maxD) and abs(minD) is the deepest penetration point.
	// ---- collisionNormal is sign(d) * planeNormal.
	// - If not, at least one of the 'd' values is zero.
	// ---- collisionNormal is sign(max(abs(maxD), abs(minD))) * planeNormal.
	double d[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double maxD = -std::numeric_limits<double>::max();
	double minD = std::numeric_limits<double>::max();
	Vector3d boxVertices[8];
	for (int i = 0; i < 8; ++i)
	{
		boxVertices[i] = boxShape.getVertex(i);
		d[i] = planeNormal.dot(boxVertices[i]) + planeD;
		maxD = std::max(d[i], maxD);
		minD = std::min(d[i], minD);
	}

	if (!(maxD > DistanceEpsilon && minD > DistanceEpsilon) && !(maxD < -DistanceEpsilon && minD < -DistanceEpsilon))
	{
		// There is an intersection.
		// Two cases:
		// - Vertex touching plane.
		// - Vertex through plane.

		Vector3d normal;

		enum BoxPlaneIntersectionType
		{
			BoxPlaneIntersectionTypeEqualsZero,
			BoxPlaneIntersectionTypeLessThanZero,
			BoxPlaneIntersectionTypeGreaterThanZero
		} boxPlaneIntersectionType;

		if (std::abs(maxD) < DistanceEpsilon)
		{
			// Box is touching the "back side" of plane.
			normal = -(planePose.linear() * planeShape.getNormal());
			boxPlaneIntersectionType = BoxPlaneIntersectionTypeEqualsZero;
		}
		else if (std::abs(minD) < DistanceEpsilon)
		{
			// Box is touching the "front side" of plane.
			normal = planePose.linear() * planeShape.getNormal();
			boxPlaneIntersectionType = BoxPlaneIntersectionTypeEqualsZero;
		}
		else
		{
			if (std::abs(maxD) >= std::abs(minD))
			{
				// Box is penetrating through the "front side" of plane.
				normal = planePose.linear() * planeShape.getNormal();
				boxPlaneIntersectionType = BoxPlaneIntersectionTypeLessThanZero;
			}
			else
			{
				// Box is penetrating through the "back side" of plane.
				normal = -(planePose.linear() * planeShape.getNormal());
				boxPlaneIntersectionType = BoxPlaneIntersectionTypeGreaterThanZero;
			}
		}

		// Loop through vertices and check if a contact point needs to be generated.
		bool generateContact = false;
		for (int i = 0; i < 8; ++i)
		{
			switch (boxPlaneIntersectionType)
			{
				case BoxPlaneIntersectionTypeEqualsZero:
					generateContact = std::abs(d[i]) < DistanceEpsilon;
					break;
				case BoxPlaneIntersectionTypeLessThanZero:
					generateContact = d[i] < -DistanceEpsilon;
					break;
				case BoxPlaneIntersectionTypeGreaterThanZero:
					generateContact = d[i] > DistanceEpsilon;
					break;
			}

			if (generateContact)
			{
				std::pair<Location, Location> penetrationPoints = std::make_pair(Location(boxVertices[i]),
						Location(planePose.inverse() * (boxPose * boxVertices[i] + normal * std::abs(d[i]))));
				contacts.emplace_back(std::make_shared<Contact>(
										  COLLISION_DETECTION_TYPE_DISCRETE,
										  std::abs(d[i]), 1.0, Vector3d::Zero(), normal,
										  penetrationPoints));

				generateContact = false;
			}
		}
	}
	return contacts;
}

}; // namespace Collision
}; // namespace SurgSim

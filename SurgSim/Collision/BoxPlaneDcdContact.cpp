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

#include "SurgSim/Collision/BoxPlaneDcdContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::BoxShape;
using SurgSim::Math::Geometry::DistanceEpsilon;
using SurgSim::Math::PlaneShape;
using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Collision
{
std::pair<int, int> BoxPlaneDcdContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_BOX, SurgSim::Math::SHAPE_TYPE_PLANE);
}

std::list<std::shared_ptr<Contact>> BoxPlaneDcdContact::calculateContact(
									 const SurgSim::Math::BoxShape& boxShape,
									 const SurgSim::Math::RigidTransform3d& boxPose,
									 const SurgSim::Math::PlaneShape& planeShape,
									 const SurgSim::Math::RigidTransform3d& planePose) const
{
	std::list<std::shared_ptr<Contact>> contacts;

	// Transform the plane normal to box co-ordinate system.
	SurgSim::Math::RigidTransform3d planeLocalToBoxLocal = boxPose.inverse() * planePose;
	SurgSim::Math::RigidTransform3d boxLocalToPlaneLocal = planeLocalToBoxLocal.inverse();
	Vector3d planeNormal = planeLocalToBoxLocal.linear() * planeShape.getNormal();
	Vector3d planeNormalScaled = planeShape.getNormal() * -planeShape.getD();
	Vector3d planePoint = planeLocalToBoxLocal * planeNormalScaled;
	double planeD = -planeNormal.dot(planePoint);

	// Loop through the box vertices (boxVertex) and check it it is below plane.
	double d = 0.0;
	Vector3d boxVertex;
	for (int i = 0; i < 8; ++i)
	{
		boxVertex = boxShape.getVertex(i);
		d = planeNormal.dot(boxVertex) + planeD;
		if (d < DistanceEpsilon)
		{
			std::pair<Location, Location> penetrationPoints = std::make_pair(Location(boxVertex),
					Location(boxLocalToPlaneLocal * (boxVertex - planeNormal * d)));
			contacts.emplace_back(std::make_shared<Contact>(
									  COLLISION_DETECTION_TYPE_DISCRETE, -d, 1.0,
									  Vector3d::Zero(), planePose.linear() * planeShape.getNormal(),
									  penetrationPoints));
		}
	}
	return contacts;
}

}; // namespace Collision
}; // namespace SurgSim

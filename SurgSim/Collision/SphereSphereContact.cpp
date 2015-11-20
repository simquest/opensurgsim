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

#include "SurgSim/Collision/SphereSphereContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

std::list<std::shared_ptr<Contact>> SphereSphereDcdContact::calculateDcdContact(
									 const Math::SphereShape& sphere1,
									 const Math::RigidTransform3d& sphere1Pose,
									 const Math::SphereShape& sphere2,
									 const Math::RigidTransform3d& sphere2Pose) const
{
	std::list<std::shared_ptr<Contact>> contacts;

	Vector3d center1 = sphere1Pose.translation();
	Vector3d center2 = sphere2Pose.translation();

	Vector3d normal = center1 - center2;
	double dist = normal.norm();
	double maxDist = sphere1.getRadius() + sphere2.getRadius();
	if (dist < maxDist)
	{
		std::pair<Location, Location> penetrationPoints;
		normal.normalize();
		penetrationPoints.first.rigidLocalPosition.setValue(
			(sphere1Pose.linear().inverse() * -normal) * sphere1.getRadius());
		penetrationPoints.second.rigidLocalPosition.setValue(
			(sphere2Pose.linear().inverse() * normal) * sphere2.getRadius());

		contacts.emplace_back(std::make_shared<Contact>(
								  COLLISION_DETECTION_TYPE_DISCRETE, maxDist - dist, 1.0,
								  Vector3d::Zero(), normal, penetrationPoints));
	}

	return contacts;
}

std::pair<int, int> SphereSphereDcdContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_SPHERE, SurgSim::Math::SHAPE_TYPE_SPHERE);
}


}; // namespace Collision
}; // namespace SurgSim

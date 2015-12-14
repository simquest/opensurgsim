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

#include "SurgSim/Collision/CapsuleSphereDcdContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/CapsuleShape.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/SphereShape.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::CapsuleShape;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

std::pair<int, int> CapsuleSphereDcdContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_CAPSULE, SurgSim::Math::SHAPE_TYPE_SPHERE);
}

std::list<std::shared_ptr<Contact>> CapsuleSphereDcdContact::calculateContact(
									 const Math::CapsuleShape& capsule, const Math::RigidTransform3d& capsulePose,
									 const Math::SphereShape& sphere, const Math::RigidTransform3d& spherePose) const
{
	std::list<std::shared_ptr<Contact>> contacts;

	Vector3d sphereCenter(spherePose.translation());
	Vector3d globalTop(capsulePose * capsule.topCenter());
	Vector3d globalBottom(capsulePose * capsule.bottomCenter());
	Vector3d result;

	double dist =
		SurgSim::Math::distancePointSegment(sphereCenter, globalTop, globalBottom, &result);
	double distThreshold = capsule.getRadius() + sphere.getRadius();

	if (dist < distThreshold)
	{
		double depth = distThreshold - dist;

		// Calculate the normal going from the sphere to the capsule
		Vector3d normal = (result - sphereCenter).normalized();

		std::pair<Location, Location> penetrationPoints;
		penetrationPoints.first.rigidLocalPosition.setValue(
			capsulePose.inverse() * (result - normal * capsule.getRadius()));
		penetrationPoints.second.rigidLocalPosition.setValue(
			spherePose.inverse() * (sphereCenter + normal * sphere.getRadius()));

		contacts.emplace_back(std::make_shared<Contact>(
								  COLLISION_DETECTION_TYPE_DISCRETE, depth, 1.0,
								  Vector3d::Zero(), normal, penetrationPoints));
	}

	return contacts;
}

}; // namespace Collision
}; // namespace SurgSim

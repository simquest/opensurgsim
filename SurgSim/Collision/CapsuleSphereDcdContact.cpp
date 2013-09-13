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

#include <SurgSim/Collision/CapsuleSphereDcdContact.h>

#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Collision/CollisionPair.h>
#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Physics/CapsuleShape.h>
#include <SurgSim/Physics/SphereShape.h>

using SurgSim::Physics::CapsuleShape;
using SurgSim::Physics::SphereShape;

namespace SurgSim
{
namespace Collision
{

void CapsuleSphereDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<CollisionRepresentation> representationCapsule(pair->getFirst());
	std::shared_ptr<CollisionRepresentation> representationSphere(pair->getSecond());

	std::shared_ptr<CapsuleShape> capsule(std::static_pointer_cast<CapsuleShape>(representationCapsule->getShape()));
	std::shared_ptr<SphereShape> sphere(std::static_pointer_cast<SphereShape>(representationSphere->getShape()));

	Vector3d sphereCenter(representationSphere->getPose().translation());
	Vector3d globalTop(representationCapsule->getPose() * capsule->topCentre());
	Vector3d globalBottom(representationCapsule->getPose() * capsule->bottomCentre());
	Vector3d result;

	double dist =
		SurgSim::Math::distancePointSegment(sphereCenter, globalTop, globalBottom, &result);
	double distThreshold = capsule->getRadius() + sphere->getRadius();

	if (dist < distThreshold)
	{
		double depth = distThreshold - dist;

		// Calculate the normal going from the sphere to the capsule
		Vector3d normal = (result - sphereCenter).normalized();

		std::pair<Location,Location> penetrationPoints;
		penetrationPoints.first.globalPosition.setValue(result - normal * capsule->getRadius());
		penetrationPoints.second.globalPosition.setValue(sphereCenter + normal * sphere->getRadius());

		pair->addContact(depth, normal, penetrationPoints);
	}
}

}; // namespace Collision
}; // namespace SurgSim
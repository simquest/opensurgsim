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

#include "SurgSim/Collision/SphereDoubleSidedPlaneDcdContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/DoubleSidedPlaneShape.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::DoubleSidedPlaneShape;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

std::list<std::shared_ptr<Contact>> SphereDoubleSidedPlaneDcdContact::calculateDcdContact(
									 const Math::SphereShape& sphere,
									 const Math::RigidTransform3d& spherePose,
									 const Math::DoubleSidedPlaneShape& plane,
									 const Math::RigidTransform3d& planePose) const
{
	std::list<std::shared_ptr<Contact>> contacts;

	Vector3d sphereCenter = spherePose.translation();

	// Move into Plane coordinate system
	Vector3d planeLocalSphereCenter = planePose.inverse() * sphereCenter;

	Vector3d result;
	double dist = Math::distancePointPlane(planeLocalSphereCenter, plane.getNormal(), plane.getD(), &result);
	double distAbsolute = std::abs(dist);
	if (distAbsolute < sphere.getRadius())
	{
		double depth = sphere.getRadius() - distAbsolute;

		// Calculate the normal going from the plane to the sphere, it is the plane normal transformed by the
		// plane pose, flipped if the sphere is behind the plane and normalize it
		Vector3d normal =
			(planePose.linear() * plane.getNormal()) * ((dist < 0.0) ? -1.0 : 1.0);

		std::pair<Location, Location> penetrationPoints;
		penetrationPoints.first.rigidLocalPosition.setValue(
			spherePose.inverse() * (sphereCenter - normal * sphere.getRadius()));
		penetrationPoints.second.rigidLocalPosition.setValue(
			planePose.inverse() * (sphereCenter - normal * distAbsolute));

		contacts.emplace_back(std::make_shared<Contact>(
								  COLLISION_DETECTION_TYPE_DISCRETE, depth, 1.0,
								  Vector3d::Zero(), normal, penetrationPoints));
	}

	return contacts;
}

std::pair<int, int> SphereDoubleSidedPlaneDcdContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_SPHERE, SurgSim::Math::SHAPE_TYPE_DOUBLESIDEDPLANE);
}
}; // namespace Collision
}; // namespace SurgSim

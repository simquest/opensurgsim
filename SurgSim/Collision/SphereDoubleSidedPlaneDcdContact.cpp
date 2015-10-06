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

SphereDoubleSidedPlaneDcdContact::SphereDoubleSidedPlaneDcdContact()
{
}

std::pair<int, int> SphereDoubleSidedPlaneDcdContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_SPHERE, SurgSim::Math::SHAPE_TYPE_DOUBLESIDEDPLANE);
}

void SphereDoubleSidedPlaneDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<Representation> representationPlane;
	std::shared_ptr<Representation> representationSphere;

	representationSphere = pair->getFirst();
	representationPlane = pair->getSecond();

	std::shared_ptr<SphereShape> sphere = std::static_pointer_cast<SphereShape>(representationSphere->getShape());
	std::shared_ptr<DoubleSidedPlaneShape> plane =
		std::static_pointer_cast<DoubleSidedPlaneShape>(representationPlane->getShape());

	Vector3d sphereCenter = representationSphere->getPose().translation();

	// Move into Plane coordinate system
	Vector3d planeLocalSphereCenter =  representationPlane->getPose().inverse() * sphereCenter;

	Vector3d result;
	double dist = SurgSim::Math::distancePointPlane(planeLocalSphereCenter, plane->getNormal(), plane->getD(),
				  &result);
	double distAbsolute = std::abs(dist);
	if (distAbsolute < sphere->getRadius())
	{
		double depth = sphere->getRadius() - distAbsolute;

		// Calculate the normal going from the plane to the sphere, it is the plane normal transformed by the
		// plane pose, flipped if the sphere is behind the plane and normalize it
		Vector3d normal =
			(representationPlane->getPose().linear() * plane->getNormal()) * ((dist < 0.0) ? -1.0 : 1.0);

		std::pair<Location, Location> penetrationPoints;
		penetrationPoints.first.rigidLocalPosition.setValue(
			representationSphere->getPose().inverse() * (sphereCenter - normal * sphere->getRadius()));
		penetrationPoints.second.rigidLocalPosition.setValue(
			representationPlane->getPose().inverse() * (sphereCenter - normal * distAbsolute));

		pair->addDcdContact(depth, normal, penetrationPoints);
	}
}

}; // namespace Collision
}; // namespace SurgSim

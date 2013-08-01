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


#include <SurgSim/Physics/ContactCalculation.h>

#include <SurgSim/Framework/Log.h>
#include <SurgSim/Physics/CollisionPair.h>
#include <SurgSim/Physics/PhysicsManagerState.h>
#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Math/RigidTransform.h>


namespace SurgSim
{
namespace Physics
{

void DefaultContactCalculation::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{


	SURGSIM_ASSERT(!m_doAssert) << "Contact calculation not implemented for pairs with types ("<<
								pair->getFirst()->getShapeType() << ", " << pair->getSecond()->getShapeType() << ").";
	SURGSIM_LOG_INFO(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Contact calculation not implemented for pairs with types (" <<
			pair->getFirst()->getShapeType() << ", " << pair->getSecond()->getShapeType() << ").";
}

// Specific calculation for pairs of Spheres

void SphereSphereDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	SURGSIM_ASSERT(pair->getFirst()->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) <<
			"First Object, wrong type of object" << pair->getFirst()->getShapeType();
	SURGSIM_ASSERT(pair->getSecond()->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) <<
			"Second Object, wrong type of object" << pair->getSecond()->getShapeType();

	std::shared_ptr<SphereShape> firstSphere = std::static_pointer_cast<SphereShape>(pair->getFirst()->getShape());
	std::shared_ptr<SphereShape> secondSphere = std::static_pointer_cast<SphereShape>(pair->getSecond()->getShape());


	Vector3d firstCenter = pair->getFirst()->getPose().translation();
	Vector3d secondCenter = pair->getSecond()->getPose().translation();

	Vector3d normal = firstCenter - secondCenter;
	double dist = normal.norm();
	double maxDist = firstSphere->getRadius() + secondSphere->getRadius();
	if (dist < maxDist)
	{
		std::pair<Location,Location> penetrationPoints;
		normal.normalize();
		penetrationPoints.first.globalPosition.setValue(firstCenter - normal * firstSphere->getRadius());
		penetrationPoints.second.globalPosition.setValue(secondCenter + normal * secondSphere->getRadius());

		pair->addContact(maxDist - dist, normal, penetrationPoints);
	}
}

void SphereDoubleSidedPlaneDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<CollisionRepresentation> representationPlane;
	std::shared_ptr<CollisionRepresentation> representationSphere;

	representationSphere = pair->getFirst();
	representationPlane = pair->getSecond();

	SURGSIM_ASSERT(representationSphere->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) <<
			"First Object, wrong type of object" << pair->getFirst()->getShapeType();
	SURGSIM_ASSERT(representationPlane->getShapeType() == RIGID_SHAPE_TYPE_DOUBLESIDEDPLANE) <<
			"Second Object, wrong type of object" << pair->getSecond()->getShapeType();

	std::shared_ptr<SphereShape> sphere = std::static_pointer_cast<SphereShape>(representationSphere->getShape());
	std::shared_ptr<DoubleSidedPlaneShape> plane =
		std::static_pointer_cast<DoubleSidedPlaneShape>(representationPlane->getShape());

	Vector3d sphereCenter = representationSphere->getPose().translation();

	// Move into Plane coordinate system
	Vector3d planeLocalSphereCenter =  representationPlane->getPose().inverse() * sphereCenter;

	Vector3d result;
	double dist = SurgSim::Math::distancePointPlane(planeLocalSphereCenter, plane->getNormal(), plane->getD(), &result);
	double distAbsolute = std::abs(dist);
	if (distAbsolute < sphere->getRadius())
	{
		double depth = sphere->getRadius() - distAbsolute;

		// Calculate the normal going from the plane to the sphere, it is the plane normal transformed by the
		// plane pose, flipped if the sphere is behind the plane and normalize it
		Vector3d normal =
			((representationPlane->getPose() * plane->getNormal()) * ((dist < 0) ? -1.0 : 1.0)).normalized();

		std::pair<Location,Location> penetrationPoints;
		penetrationPoints.first.globalPosition.setValue(sphereCenter - normal * sphere->getRadius());
		penetrationPoints.second.globalPosition.setValue(sphereCenter - normal * distAbsolute);

		pair->addContact(depth, normal, penetrationPoints);
	}
}


}; // Physics
}; // SurgSim

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
#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Math/RigidTransform.h>


namespace SurgSim
{
namespace Physics
{

void DefaultContactCalculation::calculateContact(std::shared_ptr<CollisionPair> pair)
{

	SURGSIM_ASSERT(!m_doAssert) << "Contact calculation not implemented for pairs with types ("<<
		pair->getFirst()->getShapeType() << ", " << pair->getSecond()->getShapeType() << ").";
	SURGSIM_LOG_INFO(SurgSim::Framework::Logger::getDefaultLogger()) << "Contact calculation not implemented for pairs with types ("<<
		pair->getFirst()->getShapeType() << ", " << pair->getSecond()->getShapeType() << ").";
}

void SphereSphereDcdContact::calculateContact(std::shared_ptr<CollisionPair> pair)
{
	SURGSIM_ASSERT(pair->getFirst()->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) << "First Object, wrong type of object" << 
																					pair->getFirst()->getShapeType();
	SURGSIM_ASSERT(pair->getSecond()->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) << "Second Object, wrong type of object" << 
																					pair->getSecond()->getShapeType();

	std::shared_ptr<SphereShape> leftSphere = std::static_pointer_cast<SphereShape>(pair->getFirst()->getShape());
	std::shared_ptr<SphereShape> rightSphere = std::static_pointer_cast<SphereShape>(pair->getSecond()->getShape());


	Vector3d leftCenter = pair->getFirst()->getLocalToWorldTransform().translation();
	Vector3d rightCenter = pair->getSecond()->getLocalToWorldTransform().translation();

	Vector3d normal = rightCenter - leftCenter;
	double dist = normal.norm();
	double maxDist = leftSphere->getRadius() + rightSphere->getRadius();
	if (dist < maxDist)
	{
		pair->addContact(maxDist - dist, normal.normalized());
	}
}

void SpherePlaneDcdContact::calculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<CollisionRepresentation> representationPlane;
	std::shared_ptr<CollisionRepresentation> representationSphere;

	if (! m_switchPair)
	{
		representationSphere = pair->getFirst();
		representationPlane = pair->getSecond();
	}
	else
	{
		representationSphere = pair->getSecond();
		representationPlane = pair->getFirst();
	}

	SURGSIM_ASSERT(representationSphere->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) << "First Object, wrong type of object" << 
																					pair->getFirst()->getShapeType();
	SURGSIM_ASSERT(representationPlane->getShapeType() == RIGID_SHAPE_TYPE_PLANE) << "Second Object, wrong type of object" << 
																					pair->getSecond()->getShapeType();

	std::shared_ptr<SphereShape> sphere = std::static_pointer_cast<SphereShape>(representationSphere->getShape());
	std::shared_ptr<PlaneShape> plane  = std::static_pointer_cast<PlaneShape>(representationPlane->getShape());

	Vector3d sphereCenter = representationSphere->getLocalToWorldTransform().translation();

	// Move into Plane coordinate system
	sphereCenter =  representationPlane->getLocalToWorldTransform().inverse() * sphereCenter;

	Vector3d result;
	double dist = SurgSim::Math::distancePointPlane(sphereCenter, plane->getNormal(), plane->getD(), &result);
	double distAbsolute = std::abs(dist);
	if (distAbsolute < sphere->getRadius())
	{
		double depth = sphere->getRadius() - distAbsolute;
		double factor = ((dist < 0) ? -1.0 : 1.0) * ((m_switchPair) ? -1.0 : 1.0);
		pair->addContact(depth, (representationPlane->getLocalToWorldTransform() * plane->getNormal() * factor).normalized());
	}
}




}; // Physics
}; // SurgSim

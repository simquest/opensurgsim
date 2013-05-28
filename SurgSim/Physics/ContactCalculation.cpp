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
	bool result = false;

	std::shared_ptr<SphereShape> leftSphere = std::static_pointer_cast<SphereShape>(pair->getFirst()->getShape());
	std::shared_ptr<SphereShape> rightSphere = std::static_pointer_cast<SphereShape>(pair->getSecond()->getShape());


	Vector3d leftCenter = pair->getFirst()->getLocalToWorldTransform().translation();
	Vector3d rightCenter = pair->getSecond()->getLocalToWorldTransform().translation();

	Vector3d normal = rightCenter - leftCenter;
	double dist = normal.norm();
	double maxDist = leftSphere->getRadius() + rightSphere->getRadius();
	if (dist < maxDist)
	{
		std::shared_ptr<Contact> contact = m_contactFactory->getInstance();
		contact->depth = (maxDist - dist) / 2;
		contact->contact = leftCenter + normal*0.5;
		contact->normal = normal.normalized();
		pair->addContact(contact);
	}
}




}; // Physics
}; // SurgSim

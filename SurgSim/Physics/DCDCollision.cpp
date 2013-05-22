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

#include <SurgSim/Physics/DCDCollision.h>
#include <SurgSim/Physics/CollisionRepresentation.h>
#include <SurgSim/Physics/CollisionPair.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

namespace {
	using SurgSim::Physics::RigidShape;
	using SurgSim::Physics::SphereShape;
	using SurgSim::Physics::CollisionPair;
	using SurgSim::Physics::CollisionRepresentation;
	using SurgSim::Physics::Contact;

	using SurgSim::Math::RigidTransform3d;
	using SurgSim::Math::Vector3d;

	bool intersectSphereSphere(std::shared_ptr<CollisionPair> pair)
	{
		bool result = false;
		std::shared_ptr<SphereShape> leftSphere = std::static_pointer_cast<SphereShape>(pair->getFirstCollider()->getShape());
		std::shared_ptr<SphereShape> rightSphere = std::static_pointer_cast<SphereShape>(pair->getSecondCollider()->getShape());

		Vector3d leftCenter = pair->getFirstCollider()->getLocalToWorldTransform().translation();
		Vector3d rightCenter = pair->getSecondCollider()->getLocalToWorldTransform().translation();

		Vector3d normal = rightCenter - leftCenter;
		double dist = normal.norm();
		double maxDist = leftSphere->getRadius() + rightSphere->getRadius();
		if (dist < maxDist)
		{
			result = true;
			pair->addContact(maxDist - dist, leftCenter + normal*0.5, normal.normalized());
		}
		return result;
	}
}

namespace SurgSim
{
namespace Physics
{

DCDCollision::DCDCollision(std::shared_ptr<std::vector<std::shared_ptr<CollisionPair>>> pairs) :
	m_pairs(pairs)
{

}

DCDCollision::~DCDCollision()
{

}

void DCDCollision::doUpdate(double dt)
{
	std::list<CollisionPair> result;
	auto it = m_pairs->cbegin();
	auto itEnd = m_pairs->cend();
	while (it != itEnd)
	{
		calculateContacts(*it);
		++it;
	}
}

void DCDCollision::calculateContacts(std::shared_ptr<CollisionPair> it)
{
	intersectSphereSphere(it);
}

}; // Physics
}; // SurgSim


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

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Collision
{

Representation::Representation(const std::string& name) :
	SurgSim::Framework::Representation(name)
{

}

Representation::~Representation()
{

}

std::list<std::shared_ptr<SurgSim::Collision::Contact>> Representation::getCollisionsWith(
			const std::shared_ptr<SurgSim::Collision::Representation>& collisionRepresentation) const
{
	ContactMapType collisions;
	m_collisions.get(&collisions);

	auto result = collisions.find(collisionRepresentation);
	if (std::end(collisions) == result)
	{
		static std::list<std::shared_ptr<SurgSim::Collision::Contact>> emptyList;
		return emptyList;
	}
	else
	{
		return (*result).second;
	}
}

Representation::ContactMapType Representation::getCollisions() const
{
	ContactMapType collisions;
	m_collisions.get(&collisions);

	return collisions;
}

void Representation::addCollisionWith(
	const std::shared_ptr<SurgSim::Collision::Representation>& collisionRepresentation,
	const std::shared_ptr<SurgSim::Collision::Contact>& contact)
{
	ContactMapType collisions;
	m_collisions.get(&collisions);

	collisions[collisionRepresentation].push_back(contact);
	m_collisions.set(collisions);
}

bool Representation::isCollidingWith(
	const std::shared_ptr<SurgSim::Collision::Representation>& collisionRepresentation) const
{
	ContactMapType collisions;
	m_collisions.get(&collisions);

	auto result = collisions.find(collisionRepresentation);
	return std::end(collisions) != result;
}

bool Representation::hasCollision() const
{
	ContactMapType collisions;
	m_collisions.get(&collisions);

	return !collisions.empty();
}

void Representation::clearCollisions()
{
	static ContactMapType emptyCollision;
	m_collisions.set(emptyCollision);
}

void Representation::update(const double& dt)
{

}

}; // namespace Collision
}; // namespace SurgSim

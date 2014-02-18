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

#include "SurgSim/Collision/CollisionPair.h"
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

const std::deque<std::shared_ptr<SurgSim::Collision::Contact>>& 
	Representation::getContacts(std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation)
{
	return m_contacts[collisionRepresentation->getName()];
}

const std::deque<std::shared_ptr<SurgSim::Collision::Representation>>& Representation::getColliders()
{
	return m_colliders;
}

void Representation::addCollision(std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation,
								  std::shared_ptr<SurgSim::Collision::Contact> contact)
{
	auto result = std::find(std::begin(m_colliders), std::end(m_colliders), collisionRepresentation);
	if(result != std::end(m_colliders))
	{
		m_colliders.push_back(collisionRepresentation);
	}

	m_contacts[collisionRepresentation->getName()].push_back(contact);
}

bool Representation::hasContacts() const
{
	return !m_colliders.empty() && !m_contacts.empty();
}

void Representation::clearCollisions()
{
	m_colliders.clear();
	m_contacts.clear();
}

}; // namespace Collision
}; // namespace SurgSim

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

std::list<std::shared_ptr<SurgSim::Collision::Contact>>
	Representation::getCollision(std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation) const
{
	auto result = m_collisions.find(collisionRepresentation);

	if(std::end(m_collisions) == result)
	{
		static std::list<std::shared_ptr<SurgSim::Collision::Contact>> emptyList;
		return emptyList;
	}
	else
	{
		return (*result).second;
	}
}

std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
				   std::list<std::shared_ptr<SurgSim::Collision::Contact>>> Representation::getCollisions() const
{
	return m_collisions;
}

void Representation::addCollision(std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation,
								  std::shared_ptr<SurgSim::Collision::Contact> contact)
{
	m_collisions[collisionRepresentation].push_back(contact);
}

bool Representation::isCollidingWith(std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation) const
{
	auto result = m_collisions.find(collisionRepresentation);
	return (std::end(m_collisions) == result) ? false : true;
}

bool Representation::hasCollision() const
{
	return !m_collisions.empty();
}

void Representation::clearCollision()
{
	m_collisions.clear();
}

}; // namespace Collision
}; // namespace SurgSim

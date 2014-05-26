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
	SurgSim::Framework::Representation(name),
	m_collisions(std::make_shared<SurgSim::DataStructures::BufferedValue<ContactMapType>>()),
	m_writeCollisions(m_collisions)
{
}

Representation::~Representation()
{

}

std::shared_ptr<SurgSim::DataStructures::BufferedValue<ContactMapType>>	Representation::getCollisions() const
{
	return m_collisions;
}

void Representation::addCollisionWith(
	const std::shared_ptr<SurgSim::Collision::Representation>& collisionRepresentation,
	const std::shared_ptr<SurgSim::Collision::Contact>& contact)
{
	(*m_writeCollisions)[collisionRepresentation].push_back(contact);
}

void Representation::clearCollisions()
{
	m_writeCollisions->clear();
}

void Representation::update(const double& dt)
{
	m_writeCollisions.publish();
}

}; // namespace Collision
}; // namespace SurgSim

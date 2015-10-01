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

#include "SurgSim/Collision/CollisionPair.h"

#include <numeric>

#include "SurgSim/Framework/Assert.h"

using SurgSim::DataStructures::Location;


namespace SurgSim
{
namespace Collision
{

CollisionPair::CollisionPair()
{
}

CollisionPair::CollisionPair(const std::shared_ptr<Representation>& first,
							 const std::shared_ptr<Representation>& second)
{
	setRepresentations(first, second);
}

CollisionPair::~CollisionPair()
{

}

void CollisionPair::setRepresentations(const std::shared_ptr<Representation>& first,
									   const std::shared_ptr<Representation>& second)
{
	SURGSIM_ASSERT(first != nullptr && second != nullptr) << "Collision Representation cannot be null";

	// Invalidate the current contacts
	clearContacts();
	m_representations.first = first;
	m_representations.second = second;
	m_isSwapped = false;

	if (m_representations.first == m_representations.second)
	{
		m_type = m_representations.first->getSelfCollisionType();
	}
	else if (m_representations.first->getCollisionType() == COLLISION_TYPE_NONE ||
				m_representations.second->getCollisionType() == COLLISION_TYPE_NONE)
	{
		m_type = COLLISION_TYPE_NONE;
	}
	else if (m_representations.first->getCollisionType() == COLLISION_TYPE_CONTINUOUS &&
				m_representations.second->getCollisionType() == COLLISION_TYPE_CONTINUOUS)
	{
		m_type = COLLISION_TYPE_CONTINUOUS;
	}
	else
	{
		m_type = COLLISION_TYPE_DISCRETE;
	}
}

const std::pair<std::shared_ptr<Representation>, std::shared_ptr<Representation>>&
		CollisionPair::getRepresentations() const
{
	return m_representations;
}

CollisionType CollisionPair::getType() const
{
	return m_type;
}

std::shared_ptr<Representation> CollisionPair::getFirst() const
{
	return m_representations.first;
}

std::shared_ptr<Representation> CollisionPair::getSecond() const
{
	return m_representations.second;
}

bool CollisionPair::hasContacts() const
{
	return !m_contacts.empty();
}

void CollisionPair::addContact(const double& depth, const double& time, const SurgSim::Math::Vector3d& contactPoint,
		const SurgSim::Math::Vector3d& normal, const std::pair<Location, Location>& penetrationPoints)
{
	addContact(std::make_shared<Contact>(getType(), depth, time, contactPoint, normal, penetrationPoints));
}

void CollisionPair::addContact(const double& depth, const SurgSim::Math::Vector3d& normal,
		const std::pair<Location, Location>& penetrationPoints)
{
	addContact(std::make_shared<Contact>(getType(), depth, 1.0, Math::Vector3d::Zero(), normal, penetrationPoints));
}

void CollisionPair::addContact(const std::shared_ptr<Contact>& contact)
{
	m_contacts.push_back(contact);
	m_representations.first->addContact(m_representations.second, contact);
	std::shared_ptr<Contact> contact2 =
		std::make_shared<Contact>(contact->collisionType, contact->depth, contact->time, contact->contact,
								  -contact->normal, std::pair<Location, Location>(
									  contact->penetrationPoints.second, contact->penetrationPoints.first));
	m_representations.second->addContact(m_representations.first, contact2);
}

const std::list<std::shared_ptr<Contact>>& CollisionPair::getContacts() const
{
	return m_contacts;
}

void CollisionPair::clearContacts()
{
	m_contacts.clear();
}

void CollisionPair::swapRepresentations()
{
	SURGSIM_ASSERT(! hasContacts()) << "Can't swap representations after contacts have already been calculated";
	m_isSwapped = !m_isSwapped;
	std::swap(m_representations.first, m_representations.second);
}

bool CollisionPair::isSwapped() const
{
	return m_isSwapped;
}

}; // namespace Collision
}; // namespace SurgSim


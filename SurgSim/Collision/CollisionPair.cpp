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

#include <numeric>

#include "SurgSim/Collision/CollisionPair.h"

#include "SurgSim/Collision/Representation.h"
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
							 const std::shared_ptr<Representation>& second) :
		m_representations(first, second), m_isSwapped(false)
{
	SURGSIM_ASSERT(first != second) << "Collision Representation cannot collide with itself";
	SURGSIM_ASSERT(first != nullptr && second != nullptr) << "Collision Representation cannot be null";
}

CollisionPair::~CollisionPair()
{

}

void CollisionPair::setRepresentations(const std::shared_ptr<Representation>& first,
							   const std::shared_ptr<Representation>& second)
{
	SURGSIM_ASSERT(first != second) << "Should try to collide with self";
	SURGSIM_ASSERT(first != nullptr && second != nullptr) << "Collision Representation cannot be null";

	// Invalidate the current contacts
	clearContacts();
	m_representations.first = first;
	m_representations.second = second;
	m_isSwapped = false;
}

const std::pair<std::shared_ptr<Representation>, std::shared_ptr<Representation>>&
	CollisionPair::getRepresentations() const
{
	return m_representations;
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

void CollisionPair::addContact(const double& depth,
							   const SurgSim::Math::Vector3d& contactPoint,
							   const SurgSim::Math::Vector3d& normal,
							   const std::pair<Location, Location>& penetrationPoints)
{
	addContact(std::make_shared<Contact>(depth, contactPoint, normal, penetrationPoints));
}

void CollisionPair::addContact(const double& depth,
							   const SurgSim::Math::Vector3d& normal,
							   const std::pair<Location, Location>& penetrationPoints)
{
	addContact(std::make_shared<Contact>(depth, SurgSim::Math::Vector3d::Zero(), normal, penetrationPoints));
}

void CollisionPair::addContact(const std::shared_ptr<Contact>& contact)
{
	m_contacts.push_back(contact);
	m_representations.first->addContactWith(m_representations.second, contact);
	std::shared_ptr<Contact> contact2 =
		std::make_shared<Contact>(contact->depth, contact->contact, -contact->normal,
								  std::pair<Location, Location>(
									contact->penetrationPoints.second,
									contact->penetrationPoints.first));
	m_representations.second->addContactWith(m_representations.first, contact2);
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


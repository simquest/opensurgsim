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

#ifndef SURGSIM_PHYSICS_COLLISIONPAIR_H
#define SURGSIM_PHYSICS_COLLISIONPAIR_H

#include <memory>
#include <list>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Physics/RigidActor.h>
#include <SurgSim/Physics/CollisionRepresentation.h>

namespace SurgSim
{
namespace Physics
{

/// Contact data structure used when two representations touch each other 
/// The convention is that if body 1 is moved along the normal vector by
/// a distance depth (or equivalently if body 2 is moved the same distance
/// in the opposite direction) then the penetration depth will be reduced to
/// zero. This means that the normal vector points "in" to body 1
struct Contact {
	double depth;						///< What is the penetration depth for the representation
	SurgSim::Math::Vector3d contact;	///< The actual contact point, only used for CCD
	SurgSim::Math::Vector3d normal;		///< The normal on the contact point (normalized)
};

/// Collision Pair class, it signifies a pair of items that should be checked with the
/// collision algorithm, this structure will be used for input as well as output, as contacts
/// get appended to the contacts list when found.
/// \note When used in a ReuseFactory, please note that contained contacts won't get deallocated
/// 	  until the next use, which might not be until the end of the program
class CollisionPair
{
public:

	/// Default constructor needed for ReuseFactory
	CollisionPair() {}

	/// Normal constructor
	CollisionPair(std::shared_ptr<CollisionRepresentation> first, std::shared_ptr<CollisionRepresentation> second);
	
	/// Destructor
	~CollisionPair();

	/// Sets the representations in this pair, representations cannot be the same instance and neither can be nullptr.
	/// \param	first 	The first CollisionRepresenation.
	/// \param	second	The second CollisionRepresentation.
	inline void setRepresentations(std::shared_ptr<CollisionRepresentation> first, std::shared_ptr<CollisionRepresentation> second)
	{
		SURGSIM_ASSERT(first != second) << "Should try to collide with self";
		SURGSIM_ASSERT(first != nullptr && second != nullptr) << "CollisionRepresentation cannot be null";
		
		// Invalidate the current contacts
		clearContacts();
		m_first = first;
		m_second = second;
	}

	/// \return The representation considered to be the first
	inline std::shared_ptr<CollisionRepresentation> getFirst() const
	{
		return m_first;
	}

	/// \return The representation considered to be the second
	inline std::shared_ptr<CollisionRepresentation> getSecond() const
	{
		return m_second;
	}

	/// \return true if there are any contacts assigned to the pair, false otherwise
	inline bool hasContacts() const
	{
		return !m_contacts.empty();
	}

	/// Adds a contact to the collision pair.
	/// \param	depth			The depth of the intersection.
	/// \param	contactPoint	The contact point, between the two bodies.
	/// \param	normal			The normal of the contact pointing into the first representation.
	inline void addContact(const double& depth, SurgSim::Math::Vector3d contactPoint, SurgSim::Math::Vector3d normal)
	{
		Contact contact = {depth,contactPoint,normal};
		m_contacts.push_back(std::make_shared<Contact>(contact));
	}

	/// Adds a contact to the collision pair.
	/// \param	depth			The depth of the intersection.
	/// \param	normal			The normal of the contact pointing into the first representation.
	inline void addContact(const double& depth, SurgSim::Math::Vector3d normal)
	{
		Contact contact = {depth,SurgSim::Math::Vector3d(0.0,0.0,0.0),normal};
		m_contacts.push_back(std::make_shared<Contact>(contact));
	}

	/// Adds a contact.
	/// \param	contact	The contact between the first and the second representation.
	inline void addContact(std::shared_ptr<Contact> contact)
	{
		m_contacts.push_back(contact);
	}

	/// \return	All the contacts.
	const std::list<std::shared_ptr<Contact>>& getContacts()
	{
		return m_contacts;
	}

	/// Reset clear the list of contacts, invalidating all the contacts
	void clearContacts();

private:

	/// Pair of objects that are colliding
	std::shared_ptr<CollisionRepresentation> m_first;
	std::shared_ptr<CollisionRepresentation> m_second;

	/// List of current contacts
	std::list<std::shared_ptr<Contact>> m_contacts;
};


}; // namespace physics
}; // namespace SurgSim

#endif

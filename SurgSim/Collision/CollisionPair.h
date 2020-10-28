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

#ifndef SURGSIM_COLLISION_COLLISIONPAIR_H
#define SURGSIM_COLLISION_COLLISIONPAIR_H

#include <list>
#include <memory>

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Math/Vector.h"


namespace SurgSim
{
namespace Collision
{

/// Contact data structure used when two representations touch each other
/// The convention is that if body 1 is moved along the normal vector by
/// a distance depth (or equivalently if body 2 is moved the same distance
/// in the opposite direction) then the penetration depth will be reduced to
/// zero. This means that the normal vector points "in" to body 1
struct Contact
{
	Contact(const CollisionDetectionType& newType,
			const double& newDepth,
			const double& newTime,
			const SurgSim::Math::Vector3d& newContact,
			const SurgSim::Math::Vector3d& newNormal,
			const std::pair<SurgSim::DataStructures::Location,
			SurgSim::DataStructures::Location>& newPenetrationPoints) :
		type(newType), depth(newDepth), time(newTime), contact(newContact),
		normal(newNormal), penetrationPoints(newPenetrationPoints), force(SurgSim::Math::Vector3d::Zero())
	{
	}
	std::shared_ptr<Contact> makeComplimentary()
	{
		auto complimentary = std::make_shared<Contact>(type, depth, time, contact,
							 -normal, std::make_pair(penetrationPoints.second, penetrationPoints.first));
		complimentary->force = -force;
		return complimentary;
	}
	bool operator==(const Contact& contact) const
	{
		return type == contact.type &&
			   std::abs(time - contact.time) < 1e-8 &&
			   penetrationPoints.first.isApprox(contact.penetrationPoints.first) &&
			   penetrationPoints.second.isApprox(contact.penetrationPoints.second) &&
			   normal.isApprox(contact.normal);
	}
	CollisionDetectionType type;						///< What collision algorithm class was used to get the contact
	double depth;										///< What is the penetration depth for the representation
	double time;										///< What is the time of the collision, CCD only
	SurgSim::Math::Vector3d contact;					///< The actual contact point, only used for CCD
	SurgSim::Math::Vector3d normal;						///< The normal on the contact point (normalized)
	std::pair<SurgSim::DataStructures::Location,
		SurgSim::DataStructures::Location> penetrationPoints;	///< The deepest point inside the opposing object
	SurgSim::Math::Vector3d force;						///< The reaction force to correct this contact.
};

/// Collision Pair class, it signifies a pair of items that should be checked with the
/// collision algorithm, this structure will be used for input as well as output, as contacts
/// get appended to the contacts list when found.
class CollisionPair
{
public:
	/// Default Constructor
	CollisionPair();

	/// Normal constructor
	CollisionPair(const std::shared_ptr<Representation>& first,
				  const std::shared_ptr<Representation>& second);

	/// Destructor
	~CollisionPair();

	/// Sets the representations in this pair, representations cannot be the same instance and neither can be nullptr.
	/// \param	first 	The first Collision Representation.
	/// \param	second	The second Collision Representation.
	void setRepresentations(const std::shared_ptr<Representation>& first,
							const std::shared_ptr<Representation>& second);

	/// Function that returns the pair of representations of the objects that are colliding.
	/// \return The pair of representations that are colliding.
	const std::pair<std::shared_ptr<Representation>, std::shared_ptr<Representation>>&
			getRepresentations() const;

	/// Get the collision detection type for this pair
	/// \return The collision detection type
	CollisionDetectionType getType() const;

	/// \return The representation considered to be the first
	std::shared_ptr<Representation> getFirst() const;

	/// \return The representation considered to be the second
	std::shared_ptr<Representation> getSecond() const;

	/// \return true if there are any contacts assigned to the pair, false otherwise
	bool hasContacts() const;

	/// Adds a CCD contact to the collision pair.
	/// \param	depth			The depth of the intersection.
	/// \param	time			The actual time of contact as determined by the CCD algorithm.
	/// \param	contactPoint	The contact point, between the two bodies at time "time"
	/// \param	normal			The normal of the contact pointing into the first representation.
	/// \param	penetrationPoints The points furthest into the opposing object
	void addCcdContact(const double& depth,
					   const double& time,
					   const SurgSim::Math::Vector3d& contactPoint,
					   const SurgSim::Math::Vector3d& normal,
					   const std::pair<SurgSim::DataStructures::Location,
					   SurgSim::DataStructures::Location>& penetrationPoints);

	/// Adds a DCD contact to the collision pair.
	/// \param	depth			The depth of the intersection.
	/// \param	normal			The normal of the contact pointing into the first representation.
	/// \param	penetrationPoints The points furthest into the opposing object
	void addDcdContact(const double& depth,
					   const SurgSim::Math::Vector3d& normal,
					   const std::pair<SurgSim::DataStructures::Location,
					   SurgSim::DataStructures::Location>& penetrationPoints);

	/// Adds a contact.
	/// \param	contact	The contact between the first and the second representation.
	void addContact(const std::shared_ptr<Contact>& contact);

	/// Update the representations by adding the contacts to them.
	void updateRepresentations();

	/// \return	All the contacts.
	std::list<std::shared_ptr<Contact>>& getContacts();

	/// Reset clear the list of contacts, invalidating all the contacts
	void clearContacts();

	/// Swap the representation pair so that first becomes second and second becomes first
	void swapRepresentations();

	/// Query if this the pair has been swapped from when it was constructed.
	/// \return	true if swapped, false if not.
	bool isSwapped() const;

	/// \return whether the two represenations might have an intersection
	/// \note The bounding boxes are taken, if the bounding box is empty it is always considered for collision
	bool mayIntersect() const;

private:
	/// Pair of objects that are colliding
	std::pair<std::shared_ptr<Representation>, std::shared_ptr<Representation>> m_representations;

	/// Collision detection type for this pair
	CollisionDetectionType m_type;

	/// List of current contacts
	std::list<std::shared_ptr<Contact>> m_contacts;

	bool m_isSwapped;
};


}; // namespace Collision
}; // namespace SurgSim

template <typename charT, typename traits>
std::basic_ostream<charT, traits>& operator << (std::basic_ostream<charT, traits>& out,
		const SurgSim::Collision::Contact& contact)
{
	out << "Type: " << contact.type << std::endl;
	out << "Depth: " << contact.depth << std::endl;
	out << "Time: " << contact.time << std::endl;
	out << "Contact: " << contact.contact.transpose() << std::endl;
	out << "Normal: " << contact.normal.transpose() << std::endl;
	out << "Force: " << contact.force.transpose() << std::endl;
	out << "Penetration Point 1 :" << contact.penetrationPoints.first << std::endl;
	out << "Penetration Point 2 :" << contact.penetrationPoints.second << std::endl;
	return out;
}
#endif

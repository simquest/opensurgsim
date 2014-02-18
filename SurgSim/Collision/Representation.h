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

#ifndef SURGSIM_COLLISION_REPRESENTATION_H
#define SURGSIM_COLLISION_REPRESENTATION_H

#include <deque>
#include <memory>
#include <unordered_map>

#include "SurgSim/Framework/Representation.h"

namespace SurgSim
{

namespace Math
{
class Shape;
};

namespace Physics
{
class Representation;
};

namespace Collision
{
struct Contact;

/// Wrapper class to use for the collision operation, handles its enclosed shaped
/// and a possible local to global coordinate system transform, if the physics representation
/// is a nullptr or a has gone out of scope ASSERT's will be triggered
class Representation : public SurgSim::Framework::Representation
{
public:
	/// Constructor
	/// \param name Name of this collision representation
	explicit Representation(const std::string& name);

	/// Destructor
	virtual ~Representation();

	/// Get the shape type id
	/// \return The unique type of the shape, used to determine which calculation to use.
	virtual int getShapeType() const = 0;

	/// Get the shape
	/// \return The actual shape used for collision.
	virtual const std::shared_ptr<SurgSim::Math::Shape> getShape() const = 0;

	/// Gets physics representation.
	/// \return	The physics representation.
	virtual std::shared_ptr<SurgSim::Physics::Representation> getPhysicsRepresentation() = 0;

	/// Given a collision representation, return the contact information between it and this collision representation,
	/// if any.
	/// \return A std::deque containing contact informatoin.
	const std::deque<std::shared_ptr<SurgSim::Collision::Contact>>&
		getContacts(std::shared_ptr<SurgSim::Collision::Representation>);

	/// Get the collision representations which are colliding with this collision representation.
	const std::deque<std::shared_ptr<SurgSim::Collision::Representation>>& getColliders();

	/// Add the collision information to this collision representation
	/// \param collisionRepresentation The collision representation with which this representation is colliding
	/// \param contact The contact point.
	void addCollision(std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation,
					  std::shared_ptr<SurgSim::Collision::Contact> contact);

	/// Check if this collision representation has collisions.
	/// \return True if there is a collision; otherwise false.
	bool hasContacts() const;

	/// Empty the contact list.
	void clearCollisions();

protected:
	/// A list (implemented as std::deque) of SurgSim::Collision::Representations colliding with this.
	std::deque<std::shared_ptr<SurgSim::Collision::Representation>> m_colliders;

	/// Contacts associated with each collision.
	std::unordered_map<std::string, std::deque<std::shared_ptr<SurgSim::Collision::Contact>>> m_contacts;
};


}; // namespace Collision
}; // namespace SurgSim

#endif

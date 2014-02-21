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

#include <list>
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
/// is a nullptr or a has gone out of scope ASSERT's will be triggered.
/// Collision with other representations will be updated by CollisionPair::addContact() and
/// be cleared every time DcdCollision::updatePair() makes a new CollisionPair.
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

	/// A map between collision representations and contacts.
	/// For each collision representation, it gives the list of contacts registered against this instance.
	/// \return A map with collision representations as keys and lists of contacts as the associated value.
	std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
		std::list<std::shared_ptr<SurgSim::Collision::Contact>>> getCollisions() const;

	/// Return the list of contacts between the given collision representation and this collision representation.
	/// \param collisionRepresentation The collision representation with which this collision representation collides.
	/// \return A list of contact points.
	std::list<std::shared_ptr<SurgSim::Collision::Contact>>
		getCollisionsWith(std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation) const;

	/// Add a contact against a given collision representation.
	/// \param collisionRepresentation The collision representation to which this collision representation collides.
	/// \param contact The contact information.
	void addCollisionWith(std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation,
						  std::shared_ptr<SurgSim::Collision::Contact> contact);

	/// Check if this collision representation is colliding with the given collision representation.
	/// \param collisionRepresentation The collision representation to be checked against.
	/// \return True if the two representations are colliding; otherwise, false.
	bool isCollidingWith(std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation) const;

	/// Check if this collision representation has collisions.
	/// \return True if there is a collision; otherwise false.
	bool hasCollision() const;

	/// Clear all the collisions.
	void clearCollisions();

protected:
	/// A map which associates a list of contacts with each collision representation.
	std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
					   std::list<std::shared_ptr<SurgSim::Collision::Contact>>> m_collisions;
};


}; // namespace Collision
}; // namespace SurgSim

#endif

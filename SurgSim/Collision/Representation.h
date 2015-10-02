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

#ifndef SURGSIM_COLLISION_REPRESENTATION_H
#define SURGSIM_COLLISION_REPRESENTATION_H

#include <boost/thread/mutex.hpp>
#include <list>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "SurgSim/DataStructures/BufferedValue.h"
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
class Representation;

typedef std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
		std::list<std::shared_ptr<SurgSim::Collision::Contact>>> ContactMapType;

/// The type of collision detection
enum CollisionType : SURGSIM_ENUM_TYPE;

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

	/// Set the type of collision detection to use between this representation and other representations
	/// \param type The collision detection type
	void setCollisionType(CollisionType type);

	/// Get the type of collision detection used between this representation and other representations
	/// \return The collision detection type
	CollisionType getCollisionType() const;

	/// Set the type of collision detection to use between this representation and itself
	/// \param type The collision detection type
	void setSelfCollisionType(CollisionType type);

	/// Get the type of collision detection used between this representation and itself
	/// \return The collision detection type
	CollisionType getSelfCollisionType() const;

	/// Get the shape
	/// \return The actual shape used for collision.
	virtual const std::shared_ptr<SurgSim::Math::Shape> getShape() const = 0;

	/// Get the shape, posed
	/// \return The shape transformed by the pose of this representation
	virtual const std::shared_ptr<SurgSim::Math::Shape> getPosedShape();

	/// A map between collision representations and contacts.
	/// For each collision representation, it gives the list of contacts registered against this instance.
	/// \return A map with collision representations as keys and lists of contacts as the associated value.
	SurgSim::DataStructures::BufferedValue<ContactMapType>& getCollisions();

	/// Add a contact with another representation
	/// \param other The other collision representation
	/// \param contact The contact to be added
	/// \note This method is thread-safe
	void addContact(const std::shared_ptr<Representation>& other,
		const std::shared_ptr<SurgSim::Collision::Contact>& contact);

	/// Check whether this collision representation collided with another during the last update
	/// \param other other collision representation to check against
	/// \return true if there were contacts recorded, false otherwise
	bool collidedWith(const std::shared_ptr<Representation>& other);

	/// Update the representation.
	/// \param dt the time passed from the last update.
	virtual void update(const double& dt);

	/// Set a collision representation to ignore
	/// Collisions with this collision representation will not be detected
	/// \note This method conflicts with setOnlyCollideWith. You can only set what
	/// representations to ignore or only collide with, not both.
	/// \param fullName The full name of the collision representation to ignore
	bool ignore(const std::string& fullName);

	/// Set a collision representation to ignore
	/// Collisions with this collision representation will not be detected
	/// \note This method conflicts with setOnlyCollideWith. You can only set what
	/// representations to ignore or only collide with, not both.
	/// \param representation The collision representation to ignore
	bool ignore(const std::shared_ptr<Representation>& representation);

	/// Set the collision representations to ignore
	/// Collisions with these collision representation will not be detected
	/// \note This method conflicts with setOnlyCollideWith. You can only set what
	/// representations to ignore or only collide with, not both.
	/// \param fullNames The collision representations (given by full name) to ignore
	void setIgnoring(const std::vector<std::string>& fullNames);

	/// Is the collision representation being ignored
	/// \param fullName The full name of the collision representation to check
	/// return True if the collision representation is being ignored
	bool isIgnoring(const std::string& fullName) const;

	/// Is the collision representation being ignored
	/// \param representation The collision representation to check
	/// return True if the collision representation is being ignored
	bool isIgnoring(const std::shared_ptr<Representation>& representation) const;

	/// Set the only collision representations to collide with
	/// Only Collisions with these collision representation will be detected
	/// \note This method conflicts with ignore and setIgnoring. You can only set what
	/// representations to ignore or only collide with, not both.
	/// \param fullNames The collision representations (given by full name) to only collide with
	void setOnlyCollideWith(const std::vector<std::string>& fullNames);

protected:
	/// Invalidate the cached posed shape
	void invalidatePosedShape();

	/// Get the ignored collision representations
	/// \return The full names of all the ignored collision representations
	std::vector<std::string> getIgnoring() const;

	/// Get the only collision representations that this representation can collide with
	/// \return The full names of all the collision representations to only collide with
	std::vector<std::string> getOnlyCollideWith() const;

private:
	/// The type of collision detection
	CollisionType m_collisionType;

	/// The type of self collision detection
	CollisionType m_selfCollisionType;

	/// A map which associates a list of contacts with each collision representation.
	/// Every contact added to this map follows the convention of pointing the contact normal toward this
	/// representation. And the first penetration point is on this representation.
	SurgSim::DataStructures::BufferedValue<ContactMapType> m_collisions;

	/// Mutex to lock write access to m_collisions
	boost::mutex m_collisionsMutex;

	/// Cached posed shape
	std::shared_ptr<Math::Shape> m_posedShape;

	/// Mutex to lock write access to m_posedShape
	boost::mutex m_posedShapeMutex;

	/// Pose of m_posedShape
	Math::RigidTransform3d m_posedShapePose;

	/// Ignored collision representations
	std::unordered_set<std::string> m_ignoring;

	/// Collision representations to only collide with
	std::unordered_set<std::string> m_onlyCollideWith;
};

}; // namespace Collision
}; // namespace SurgSim

SURGSIM_SERIALIZABLE_ENUM(SurgSim::Collision::CollisionType,
	(COLLISION_TYPE_NONE)
	(COLLISION_TYPE_DISCRETE)
	(COLLISION_TYPE_CONTINUOUS)
	(MAX_COLLISION_TYPES)
)

#endif

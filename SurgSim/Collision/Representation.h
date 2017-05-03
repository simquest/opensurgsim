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
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Representation.h"
#include "SurgSim/Math/Aabb.h"
#include "SurgSim/Math/Shape.h"

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
enum CollisionDetectionType : SURGSIM_ENUM_TYPE;

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
	void setCollisionDetectionType(CollisionDetectionType type);

	/// Get the type of collision detection used between this representation and other representations
	/// \return The collision detection type
	CollisionDetectionType getCollisionDetectionType() const;

	/// Set the type of collision detection to use between this representation and itself
	/// \param type The collision detection type
	void setSelfCollisionDetectionType(CollisionDetectionType type);

	/// Get the type of collision detection used between this representation and itself
	/// \return The collision detection type
	CollisionDetectionType getSelfCollisionDetectionType() const;

	/// Get the shape
	/// \return The actual shape used for collision.
	virtual std::shared_ptr<SurgSim::Math::Shape> getShape() const = 0;

	/// Get the shape, posed
	/// \return The shape transformed by the pose of this representation
	virtual std::shared_ptr<SurgSim::Math::Shape> getPosedShape();

	/// \return the posed shape motion
	const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>>& getPosedShapeMotion() const;

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

	/// Update the basic Shape's state from the physics state, so that the bounding box can correctly be determined
	virtual void updateShapeData();

	/// Update the data (the shape) in preparation for a DCD contact calculation
	virtual void updateDcdData();

	/// Update the data (the motionShape) in preparation for a CCD contact calcul  ation
	/// \param timeOfImpact the last time of impact, the representation is responsible for managing
	/// the time correctly
	virtual void updateCcdData(double timeOfImpact);

	/// Set a collision representation to ignore
	/// Collisions with this collision representation will not be detected
	/// This acts as the opposite of allow if the representation that is passed here was previously added via allow()
	/// \param fullName The full name of the collision representation to ignore
	bool ignore(const std::string& fullName);

	/// Set a collision representation to ignore
	/// Collisions with this collision representation will not be detected
	/// This acts as the opposite of allow if the representation that is passed here was previously added via allow()
	/// \param representation The collision representation to ignore
	bool ignore(const std::shared_ptr<Representation>& representation);

	/// Set the collision representations to ignore
	/// Collisions with these collision representation will not be detected
	/// \note This method conflicts with setAllowing. You can only set what
	/// representations to ignore or allow collisions with, not both.
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

	/// Set a collision representation to allow
	/// Only collisions with "allowed" collision representation will be detected
	/// If the the representation is currently being "ignored" then it will be removed from that state and
	/// collisions will be allowed again.
	/// \note When both the allow and ignore lists are empty calling allow may cause a change of behavior that
	/// might not be wanted (i.e. the representation will go from colliding with all others to just colliding with
	/// one other representation). This might be caused by trying to revert an "ignore" that has already been reversed.
	/// \param fullName The full name of the collision representation to allow
	bool allow(const std::string& fullName);

	/// Set a collision representation to allow
	/// Only collisions with "allowed" collision representation will be detected
	/// If the the representation is currently being "ignored" then it will be removed from that state and
	/// collisions will be allowed again.
	/// \note When both the allow and ignore lists are empty calling allow may cause a change of behavior that
	/// might not be wanted (i.e. the representation will go from colliding with all others to just colliding with
	/// one other representation). This might be caused by trying to revert an "ignore" that has already been reversed.
	/// \param representation The collision representation to allow
	bool allow(const std::shared_ptr<Representation>& representation);

	/// Set the only collision representations to allow collisions with
	/// Only Collisions with these collision representation will be detected
	/// \note This method conflicts with ignore and setIgnoring. You can only set what
	/// representations to ignore or allow collisions with, not both.
	/// \param fullNames The collision representations (given by full name) to allow
	void setAllowing(const std::vector<std::string>& fullNames);

	/// Is the collision representation being allowed
	/// \param fullName The full name of the collision representation to check
	/// return True if the collision representation is being allowed
	bool isAllowing(const std::string& fullName) const;

	/// Is the collision representation being allowed
	/// \param representation The collision representation to check
	/// return True if the collision representation is being allowed
	bool isAllowing(const std::shared_ptr<Representation>& representation) const;

	/// \return the Bounding box for this object
	Math::Aabbd getBoundingBox() const;

protected:
	/// Invalidate the cached posed shape motion
	void invalidatePosedShapeMotion();

	/// Get the ignored collision representations
	/// \return The full names of all the ignored collision representations
	std::vector<std::string> getIgnoring() const;

	/// Get the only collision representations that this representation is allowed to collide with
	/// \return The full names of all the collision representations to allow
	std::vector<std::string> getAllowing() const;

	void doRetire() override;

	/// \param posedShape the posed shape motion to be set
	void setPosedShapeMotion(const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>>& posedShape);

	std::shared_ptr<Framework::Logger> m_logger;

private:
	/// The type of collision detection
	CollisionDetectionType m_collisionDetectionType;

	/// The type of self collision detection
	CollisionDetectionType m_selfCollisionDetectionType;

	/// A map which associates a list of contacts with each collision representation.
	/// Every contact added to this map follows the convention of pointing the contact normal toward this
	/// representation. And the first penetration point is on this representation.
	SurgSim::DataStructures::BufferedValue<ContactMapType> m_collisions;

	/// Mutex to lock write access to m_collisions
	boost::mutex m_collisionsMutex;

	/// The shape transformed in space and defined through time, i.e. with 2 differents configurations
	Math::PosedShapeMotion<std::shared_ptr<Math::Shape>> m_posedShapeMotion;

	/// Mutex to lock write access to m_posedShapeMotion
	mutable boost::shared_mutex m_posedShapeMotionMutex;

	/// Ignored collision representations
	std::unordered_set<std::string> m_ignoring;

	/// Allowed collision representations
	std::unordered_set<std::string> m_allowing;
};

}; // namespace Collision
}; // namespace SurgSim

SURGSIM_SERIALIZABLE_ENUM(SurgSim::Collision::CollisionDetectionType,
						  (COLLISION_DETECTION_TYPE_NONE)
						  (COLLISION_DETECTION_TYPE_DISCRETE)
						  (COLLISION_DETECTION_TYPE_CONTINUOUS)
						  (MAX_COLLISION_DETECTION_TYPES))

#endif

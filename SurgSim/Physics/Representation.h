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

#ifndef SURGSIM_PHYSICS_REPRESENTATION_H
#define SURGSIM_PHYSICS_REPRESENTATION_H

#include <string>

#include "SurgSim/Framework/Representation.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace DataStructures
{
struct Location;
}

namespace Collision
{
class Representation;
}

namespace Physics
{

class Localization;

/// The Representation class defines the base class for all physics objects
class Representation : public SurgSim::Framework::Representation
{
public:
	/// Constructor
	/// \param name The representation's name
	explicit Representation(const std::string& name);

	/// Destructor
	virtual ~Representation();

	/// Query the representation type
	/// \return the class name for this representation
	virtual std::string getType() const = 0;

	/// Reset the representation to its initial/default state
	virtual void resetState();

	/// Reset the representation parameters to their initial/default values
	virtual void resetParameters();

	/// Query the object number of degrees of freedom
	/// \return The number of degrees of freedom
	size_t getNumDof() const;

	/// Set the gravity enable flag
	/// \param isGravityEnabled True if gravity enabled, false if not.
	void setIsGravityEnabled(bool isGravityEnabled);

	/// Get the gravity enable flag
	/// \return true if gravity enabled, false if not.
	bool isGravityEnabled() const;

	/// Set whether this Representation is controlling the pose of the SceneElement
	/// that it is part of.
	/// \param isDrivingSceneElementPose true if this Representation is driving the pose of the SceneElement
	void setIsDrivingSceneElementPose(bool isDrivingSceneElementPose);

	/// Query if this Representation is controlling the pose of the SceneElement
	/// that it is part of.
	/// \return true if this Representation is controlling the pose of the SceneElement
	bool isDrivingSceneElementPose();

	/// Preprocessing done before the update call
	/// This needs to be called from the outside usually from a Computation
	/// \param dt The time step (in seconds)
	virtual void beforeUpdate(double dt);

	/// Update the representation state to the current time step
	/// \param dt The time step (in seconds)
	virtual void update(double dt);

	/// Postprocessing done after the update call
	/// This needs to be called from the outside usually from a Computation
	/// \param dt The time step (in seconds)
	virtual void afterUpdate(double dt);

	/// Computes a localized coordinate w.r.t this representation, given a Location object.
	/// \param location A location in 3d space.
	/// \return A localization object for the given location.
	virtual std::shared_ptr<Localization> createLocalization(const SurgSim::DataStructures::Location& location);

	/// Update the Representation's current position and velocity using a time interval, dt, and change in velocity,
	/// deltaVelocity.
	///
	/// This function typically is called in the physics pipeline (PhysicsManager::doUpdate) after solving the equations
	/// that enforce constraints when collisions occur.  Specifically it is called in the PushResults::doUpdate step.
	/// \param dt The time step
	/// \param deltaVelocity The block of a vector containing the correction to be applied to the velocity
	virtual void applyCorrection(double dt, const Eigen::VectorBlock<SurgSim::Math::Vector>& deltaVelocity);

	/// \return the collision representation for this physics representation.
	std::shared_ptr<SurgSim::Collision::Representation> getCollisionRepresentation() const;

	/// Set the collision representation for this physics representation, when the collision object
	/// is involved in a collision, the collision should be resolved inside the dynamics calculation.
	/// \param representation The appropriate collision representation for this object.
	virtual void setCollisionRepresentation(std::shared_ptr<SurgSim::Collision::Representation> representation);

protected:
	/// Set the number of degrees of freedom
	/// \param numDof The number of degrees of freedom
	/// \note protected so that nobody can change the number of DOF
	/// \note except daughter classes
	void setNumDof(size_t numDof);

	/// Get the gravity used by this Representation
	/// \return The gravity vector
	const SurgSim::Math::Vector3d& getGravity() const;

	/// This entity's collision representation, these are usually very specific to the physics representation
	std::shared_ptr<SurgSim::Collision::Representation> m_collisionRepresentation;

	/// This conditionally updates that pose for the scenelement to the given pose
	/// The update gets exectuded if the representation actually has  sceneelement and isDrivingScenElement() is true
	/// \param pose New pose for the SceneElement
	void driveSceneElementPose(const SurgSim::Math::RigidTransform3d& pose);

private:
	/// NO copy constructor
	Representation(const Representation&);

	/// NO assignment operator
	Representation& operator =(const Representation&);

	/// Gravity vector
	const SurgSim::Math::Vector3d m_gravity;

	/// Number of degrees of freedom for this representation
	size_t m_numDof;

	/// Gravity enabled flag
	bool m_isGravityEnabled;

	/// Is this representation driving the sceneElement pose
	bool m_isDrivingSceneElementPose;

};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_REPRESENTATION_H

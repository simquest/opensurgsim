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

#ifndef SURGSIM_PHYSICS_ACTOR_H
#define SURGSIM_PHYSICS_ACTOR_H

#include <string>

#include <SurgSim\Math\Vector.h>
#include <SurgSim\Framework\Representation.h>

namespace SurgSim
{

namespace Physics
{

/// The Actor class defines the base class for all physics objects
class Actor : public SurgSim::Framework::Representation
{
public:
	/// Constructor
	/// \param name The actor's name
	explicit Actor(const std::string& name)
		: Representation(name), m_numDof(0)
	{
		Actor::resetState();
	}

	/// Destructor
	virtual ~Actor()
	{
	}

	/// Reset the actor to its initial/default state
	/// The actor is being active with gravity enabled, set to (0 -9.81 0)
	virtual void resetState()
	{
		setIsActive(true);
		setIsGravityEnabled(true);
		setGravity(SurgSim::Math::Vector3d(0, -9.81, 0));
	}

	/// Reset the actor parameters to their initial/default values
	virtual void resetParameters()
	{
	}

	/// Query the object number of degrees of freedom
	/// \return The number of degrees of freedom
	unsigned int getNumDof() const
	{
		return m_numDof;
	}

	/// Set active flag for this Actor
	/// \param isActive True if the Actor is being activated, False otherwise
	virtual void setIsActive(bool isActive)
	{
		m_isActive = isActive;
	}

	/// Query if this object is active in the scene.
	/// \return true if active, false if not.
	bool isActive() const
	{
		return m_isActive;
	}

	/// Set the gravity enable flag
	/// \param isGravityEnabled True if gravity enabled, false if not.
	virtual void setIsGravityEnabled(bool isGravityEnabled)
	{
		m_isGravityEnabled = isGravityEnabled;
	}

	/// Get the gravity enable flag
	/// \return true if gravity enabled, false if not.
	bool isGravityEnabled() const
	{
		return m_isGravityEnabled;
	}

	/// Set gravity vector
	/// \param gravity The gravity vector (direction and magnitude)
	virtual void setGravity(const SurgSim::Math::Vector3d& gravity)
	{
		m_gravity = gravity;
	}

	/// Get gravity vector
	/// \return The gravity vector (direction and magnitude)
	///         No matter what the gravity enabled flag is
	const SurgSim::Math::Vector3d& getGravity() const
	{
		return m_gravity;
	}

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	virtual void beforeUpdate(double dt)
	{
	}

	/// Update the actor state to the current time step
	/// \param dt The time step (in seconds)
	virtual void update(double dt)
	{
	}

	/// Postprocessing done after the update call
	/// \param dt The time step (in seconds)
	virtual void afterUpdate(double dt)
	{
	}

protected:
	/// Set the number of degrees of freedom
	/// \param numDof The number of degrees of freedom
	/// \note protected so that nobody can change the number of DOF
	/// \note except daughter classes
	void setNumDof(unsigned int numDof)
	{
		m_numDof = numDof;
	}

private:
	/// NO copy constructor
	Actor(const Actor& a);
	
	/// NO assignement operator
	Actor& operator =(const Actor &a);

	/// Number of degrees of freedom for this actor
	unsigned int m_numDof;

	/// Gravity vector
	SurgSim::Math::Vector3d m_gravity;

	/// Gravity enabled flag
	bool m_isGravityEnabled;

	/// Is this actor active or not ?
	bool m_isActive;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_ACTOR_H

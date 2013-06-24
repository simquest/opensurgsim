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

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Framework/Representation.h>

namespace SurgSim
{

namespace Physics
{

/// The Representation class defines the base class for all physics objects
class Representation : public SurgSim::Framework::Representation
{
public:
	/// Constructor
	/// \param name The representation's name
	explicit Representation(const std::string& name)
		: SurgSim::Framework::Representation(name), m_gravity(0.0, -9.81, 0.0), m_numDof(0),
		m_isGravityEnabled(true), m_isActive(true)
	{
	}

	/// Destructor
	virtual ~Representation()
	{
	}

	/// Reset the representation to its initial/default state
	virtual void resetState()
	{
	}

	/// Reset the representation parameters to their initial/default values
	virtual void resetParameters()
	{
	}

	/// Query the object number of degrees of freedom
	/// \return The number of degrees of freedom
	unsigned int getNumDof() const
	{
		return m_numDof;
	}

	/// Set active flag for this Representation
	/// \param isActive True if the Representation is being activated, False otherwise
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

	/// Set the pose of the physics representation
	/// \param pose The pose to request the representation to be at
	/// \note setPose should be called only once per time step!
	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose) = 0;

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	virtual void beforeUpdate(double dt)
	{
	}

	/// Update the representation state to the current time step
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

	/// Get the gravity used by this Representation
	/// \return The gravity vector
	const SurgSim::Math::Vector3d& getGravity() const
	{
		return m_gravity;
	}

private:
	/// NO copy constructor
	Representation(const Representation& a);

	/// NO assignement operator
	Representation& operator =(const Representation &a);

	/// Gravity vector
	const SurgSim::Math::Vector3d m_gravity;

	/// Number of degrees of freedom for this representation
	unsigned int m_numDof;

	/// Gravity enabled flag
	bool m_isGravityEnabled;

	/// Is this representation active or not ?
	bool m_isActive;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_REPRESENTATION_H

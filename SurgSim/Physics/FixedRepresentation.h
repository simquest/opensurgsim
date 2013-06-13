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

#ifndef SURGSIM_PHYSICS_FIXEDREPRESENTATION_H
#define SURGSIM_PHYSICS_FIXEDREPRESENTATION_H

#include <SurgSim/Physics/RigidRepresentationBase.h>
#include <SurgSim/Physics/RigidRepresentationBaseState.h>

namespace SurgSim
{

namespace Physics
{

/// The FixedRepresentation class represents a physics entity without any motion nor
/// compliance against which others physics entities can interact
class FixedRepresentation : public RigidRepresentationBase
{
public:
	/// Constructor
	/// \param name The fixed representation's name
	explicit FixedRepresentation(const std::string& name)
		: RigidRepresentationBase(name)
	{
	}

	/// Destructor
	virtual ~FixedRepresentation()
	{
	}

	/// Set the initial pose of the rigid representation
	/// \param pose The initial pose (translation + rotation)
	void setInitialPose(const RigidTransform3d& pose)
	{
		m_initialState.setPose(pose);
		m_currentState   = m_initialState;
		m_previousState  = m_initialState;
	}

	/// Get the initial pose of the rigid representation
	/// \return The initial pose (translation + rotation)
	const RigidTransform3d& getInitialPose() const
	{
		return m_initialState.getPose();
	}

	/// Set the current pose of the rigid representation
	/// \param pose The current pose (translation + rotation)
	void setPose(const RigidTransform3d& pose)
	{
		m_previousState = m_currentState;
		m_currentState.setPose(pose);
	}

	/// Get the previous pose of the rigid representation
	/// \return The previous pose (translation + rotation)
	const RigidTransform3d& getPreviousPose() const
	{
		return m_previousState.getPose();
	}

	/// Get the final pose of the rigid representation
	/// \return The final pose (translation + rotation)
	const RigidTransform3d& getPose() const
	{
		return m_currentState.getPose();
	}

	/// Called to reset the fixed object to its initial/default state
	/// \post all states are set to the initial state
	void resetState()
	{
		Representation::resetState();

		m_previousState  = m_initialState;
		m_currentState   = m_initialState;
	}

private:
	/// Initial fixed representation state
	RigidRepresentationBaseState m_initialState;

	/// Previous fixed representation state
	RigidRepresentationBaseState m_previousState;

	/// Current fixed representation state
	RigidRepresentationBaseState m_currentState;
};

}; // Physics

}; // SurgSim

#endif /// SURGSIM_PHYSICS_FIXEDREPRESENTATION_H

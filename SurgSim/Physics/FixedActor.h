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

#ifndef SURGSIM_PHYSICS_FIXEDACTOR_H
#define SURGSIM_PHYSICS_FIXEDACTOR_H

#include <SurgSim/Physics/RigidActorBase.h>
#include <SurgSim/Physics/RigidActorBaseState.h>

namespace SurgSim
{

namespace Physics
{

/// The FixedActor class represents a physics entity without any motion nor
/// compliance against which others physics entities can interact
class FixedActor : public RigidActorBase
{
public:
	/// Constructor
	/// \param name The fixed actor's name
	explicit FixedActor(const std::string& name)
		: RigidActorBase(name)
	{
	}

	/// Destructor
	virtual ~FixedActor()
	{
	}

	/// Set the initial pose of the rigid actor
	/// \param pose The initial pose (translation + rotation)
	void setInitialPose(const RigidTransform3d& pose)
	{
		m_initialState.setPose(pose);
		m_currentState   = m_initialState;
		m_previousState  = m_initialState;
		m_lastValidState = m_initialState;
	}

	/// Get the initial pose of the rigid actor
	/// \return The initial pose (translation + rotation)
	const RigidTransform3d& getInitialPose() const
	{
		return m_initialState.getPose();
	}

	/// Set the current pose of the rigid actor
	/// \param pose The current pose (translation + rotation)
	void setCurrentPose(const RigidTransform3d& pose)
	{
		m_currentState.setPose(pose);
	}

	/// Get the previous pose of the rigid actor
	/// \return The previous pose (translation + rotation)
	const RigidTransform3d& getPreviousPose() const
	{
		return m_previousState.getPose();
	}

	/// Get the final pose of the rigid actor
	/// \return The final pose (translation + rotation)
	const RigidTransform3d& getFinalPose() const
	{
		return m_finalState.getPose();
	}

	/// Called prior to update
	/// \param dt The time step (in seconds)
	void beforeUpdate(double dt)
	{
		m_previousState = m_currentState;
	}

	void afterUpdate(double dt)
	{
		m_finalState = m_currentState;
	}

	/// Called to reset the fixed object to its initial/default state
	/// \post all states are set to the initial state
	void resetState()
	{
		Actor::resetState();

		m_previousState  = m_initialState;
		m_currentState   = m_initialState;
		m_finalState     = m_initialState;
	}

private:
	/// Initial fixed actor state
	RigidActorBaseState m_initialState;

	/// Previous fixed actor state
	RigidActorBaseState m_previousState;

	/// Current fixed actor state
	RigidActorBaseState m_currentState;

	/// Last valid/final actor state
	RigidActorBaseState m_finalState;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_FIXEDACTOR_H

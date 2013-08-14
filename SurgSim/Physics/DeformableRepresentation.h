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

#ifndef SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H
#define SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H

#include <SurgSim\Physics\Representation.h>
#include <SurgSim\Physics\DeformableRepresentationState.h>

namespace SurgSim
{

namespace Physics
{

/// Base class for all deformable representations MassSprings, Finite Element Models,...
/// It holds the representation states (common to all deformable) and handles state set/get/reset
class DeformableRepresentation : public Representation
{
public:
	/// Constructor
	/// \param name The deformable representation's name
	explicit DeformableRepresentation(const std::string& name)
		: Representation(name)
	{
	}

	/// Destructor
	virtual ~DeformableRepresentation()
	{
	}

	/// Set the initial pose of the representation
	/// \param pose The initial pose
	/// \note Pose is not being used for a deformable object
	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& pose) override
	{
	}

	/// Get the initial pose of the representation
	/// \return The initial pose
	/// \note Pose is not being used for a deformable object
	/// \note Therefore we return Identity
	virtual const SurgSim::Math::RigidTransform3d& getInitialPose() const override
	{
		static auto staticLocalId = SurgSim::Math::RigidTransform3d::Identity();
		return staticLocalId;
	}

	/// Set the current pose of the representation
	/// \param pose The current pose
	/// \note Pose is not being used for a deformable object
	void setPose(const SurgSim::Math::RigidTransform3d& pose) override
	{
	}

	/// Get the current pose of the representation
	/// \return The current pose
	/// \note Pose is not being used for a deformable object
	/// \note Therefore we return an identity transform
	const SurgSim::Math::RigidTransform3d& getPose() const
	{
		static auto staticLocalId = SurgSim::Math::RigidTransform3d::Identity();
		return staticLocalId;
	}

	/// Called to reset the Representation state to its initial state
	virtual void resetState() override
	{
		Representation::resetState();

		m_currentState  = m_initialState;
		m_previousState = m_initialState;
		m_finalState    = m_initialState;
	}


	/// Set the initial state
	/// \param state The initial state for this deformable actor
	/// \note This method sets the current/previous states to the initial state as well
	void setInitialState(const DeformableRepresentationState& state)
	{
		m_initialState  = state;
		m_currentState  = state;
		m_previousState = state;
		m_finalState    = state;
	}

	/// Get the initial state
	/// \return The initial state of this deformable representation
	const DeformableRepresentationState& getInitialState() const
	{
		return m_initialState;
	}

	/// Get the current state
	/// \return The current state of this deformable representation
	/// \note We are sending the current valid state, which is m_finalState
	/// \note m_currentState and m_previousState are internal states
	const DeformableRepresentationState& getCurrentState() const
	{
		return m_finalState;
	}

protected:
	/// Initial state (used for reseting the state)
	DeformableRepresentationState m_initialState;

	/// Previous state (internal use)
	DeformableRepresentationState m_previousState;

	/// Current state (internal use)
	DeformableRepresentationState m_currentState;

	/// Final state (last valid state)
	DeformableRepresentationState m_finalState;

private:
	/// NO copy constructor
	DeformableRepresentation(const DeformableRepresentation& a);

	/// NO assignment operator
	DeformableRepresentation& operator =(const DeformableRepresentation& a);
};

}; // namespace Physics

}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H

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

#include <SurgSim/Physics/Representation.h>
#include <SurgSim/Physics/DeformableRepresentationState.h>

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
	explicit DeformableRepresentation(const std::string& name);

	/// Destructor
	virtual ~DeformableRepresentation();

	/// Set the initial pose of the representation (used to transform the mesh on loading)
	/// \param pose The initial pose
	/// \note This is a feature to place the object in the scene on loading
	/// \note A deformable representation is expressed in global frame (local frame = global frame)
	/// \note This method needs to be called prior to loading the mesh
	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& pose) override;

	/// Get the initial pose of the representation
	/// \return The initial pose used to transform the loaded mesh
	virtual const SurgSim::Math::RigidTransform3d& getInitialPose() const override;

	/// Set the current pose of the representation
	/// \param pose The current pose
	/// \note A deformable representation is expressed in global frame (local frame = global frame)
	/// \note This method should not be called, as a deformable pose cannot be set !
	void setPose(const SurgSim::Math::RigidTransform3d& pose) override;

	/// Get the current pose of the representation
	/// \return The current pose (Identity)
	/// \note A deformable representation is expressed in global frame (local frame = global frame)
	/// \note Therefore its pose is always Identity
	const SurgSim::Math::RigidTransform3d& getPose() const;

	/// Called to reset the Representation state to its initial state
	virtual void resetState() override;

	/// Get the initial state (in the global frame)
	/// \return The initial state of this deformable representation
	const DeformableRepresentationState& getInitialState() const;

	/// Get the current state (in the global frame)
	/// \return The current state of this deformable representation
	const DeformableRepresentationState& getCurrentState() const;

	/// Get the previous state (in the global frame)
	/// \return The previous state of this deformable representation
	const DeformableRepresentationState& getPreviousState() const;

	/// Get the final state (in the global frame)
	/// \return The final state of this deformable representation
	const DeformableRepresentationState& getFinalState() const;

protected:
	/// Initial state (used for reseting the state)
	DeformableRepresentationState m_initialState;

	/// Previous state (internal use)
	DeformableRepresentationState m_previousState;

	/// Current state (internal use)
	DeformableRepresentationState m_currentState;

	/// Final state (last valid state)
	DeformableRepresentationState m_finalState;

	/// Initial pose that will transform the mesh on initialization
	SurgSim::Math::RigidTransform3d m_initialPose;

private:
	/// NO copy constructor
	DeformableRepresentation(const DeformableRepresentation& a);

	/// NO assignment operator
	DeformableRepresentation& operator =(const DeformableRepresentation& a);
};

}; // namespace Physics

}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H

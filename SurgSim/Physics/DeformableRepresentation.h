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

/// \file
/// Base class for all deformable representations (abstract class)

#ifndef SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H
#define SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H

#include <memory>

#include <SurgSim/Physics/Representation.h>
#include <SurgSim/Physics/DeformableRepresentationState.h>

namespace SurgSim
{

namespace Physics
{

/// Base class for all deformable representations MassSprings, Finite Element Models,...
/// \note It holds the representation states (common to all deformable)
/// \note It holds the initial pose (the pose is always identity and therefore cannot be set)
/// \note   -> the initial pose should be set before setting the initial state so the state can be properly transformed
/// \note It holds the force vector; the mass, damping and stiffness matrices (templated type)
template <class MType, class DType, class KType>
class DeformableRepresentation : public Representation
{
public:
	/// Constructor
	/// \param name The deformable representation's name
	explicit DeformableRepresentation(const std::string& name);

	/// Destructor
	virtual ~DeformableRepresentation();

	/// Set the initial pose of the representation (used to transform the initial state on setup)
	/// \param pose The initial pose
	/// \note This is a feature to place the object in the scene on loading
	/// \note A deformable representation is expressed in global frame (local frame = global frame)
	/// \note This method needs to be called prior to calling setInitialState to be effective
	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& pose) override;

	/// Get the initial pose of the representation
	/// \return The initial pose used to transform the loaded state
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

	/// Set the initial state (in the global frame)
	/// \param initialState The initial state for this deformable representation
	/// \note All states are initialized with initialState as well to make the simulation ready
	/// \note All states are transformed by the initialPose, if any as been specified prior to calling this method
	void setInitialState(std::shared_ptr<DeformableRepresentationState> initialState);

	/// Get the initial state (in the global frame)
	/// \return The initial state of this deformable representation
	const std::shared_ptr<DeformableRepresentationState> getInitialState() const;

	/// Get the current state (in the global frame)
	/// \return The current state of this deformable representation
	const std::shared_ptr<DeformableRepresentationState> getCurrentState() const;

	/// Get the previous state (in the global frame)
	/// \return The previous state of this deformable representation
	const std::shared_ptr<DeformableRepresentationState> getPreviousState() const;

	/// Get the final state (in the global frame)
	/// \return The final state of this deformable representation
	const std::shared_ptr<DeformableRepresentationState> getFinalState() const;

protected:
	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	virtual void transformState(std::shared_ptr<DeformableRepresentationState> state,
		const SurgSim::Math::RigidTransform3d& transform) = 0;

	/// Initial state (used for reseting the state), and final state (last valid state)
	std::shared_ptr<DeformableRepresentationState> m_initialState, m_finalState;

	/// Previous, current and new states (internal use)
	/// New state is a temporary variable to store the newly computed state
	std::shared_ptr<DeformableRepresentationState> m_previousState, m_currentState, m_newState;

	/// Initial pose that will transform the state on setup
	SurgSim::Math::RigidTransform3d m_initialPose;

	/// Identity pose (to avoid a static variable in the method getPose)
	SurgSim::Math::RigidTransform3d m_identityPose;

	/// Force applied on the deformable representation
	Vector m_f;

	/// Mass matrix (templatized type for performance reason)
	MType m_M;
	
	/// Damping matrix (templatized type for performance reason)
	DType m_D;
	
	/// Stiffness matrix (templatized type for performance reason)
	KType m_K;

private:
	/// NO copy constructor
	DeformableRepresentation(const DeformableRepresentation& a);

	/// NO assignment operator
	DeformableRepresentation& operator =(const DeformableRepresentation& a);
};

}; // namespace Physics

}; // namespace SurgSim

#include <SurgSim/Physics/DeformableRepresentation-inl.h>

#endif // SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H

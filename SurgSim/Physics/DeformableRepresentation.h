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

#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Math/OdeEquation.h"
#include "SurgSim/Math/OdeSolver.h"

using SurgSim::Math::OdeEquation;
using SurgSim::Math::OdeSolver;

namespace SurgSim
{

namespace Physics
{

/// Base class for all deformable representations MassSprings, Finite Element Models,...
/// \note It is both a Physics::Representation and a Math::OdeEquation
/// \note It holds the representation states (common to all deformable) except the initial state,
/// \note   which is being held by the OdeEquation (initial condition of the ode problem).
/// \note It holds the initial pose, which should be set before setting the initial state so the states
/// \note   can be properly transformed.
/// \note The current pose is always identity and therefore cannot be set. Calling setPose will raise an exception.
/// \note It holds the force vector; the mass, damping and stiffness matrices (templated type)
/// \note Derived classes must implement the Representation API and the OdeEquation API, also set
/// \note   m_numDofPerNode and call Representation::setNumDof()
template <class MType, class DType, class KType, class SType>
class DeformableRepresentation :
	public Representation,
	public OdeEquation<DeformableRepresentationState, MType, DType, KType, SType>
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
	/// \note Calling this method will raise an exception
	void setPose(const SurgSim::Math::RigidTransform3d& pose) override;

	/// Get the current pose of the representation
	/// \return The current pose (Identity)
	/// \note A deformable representation is expressed in global frame (local frame = global frame)
	/// \note Therefore its pose is always Identity
	const SurgSim::Math::RigidTransform3d& getPose() const;

	/// Called to reset the Representation state to its initial state
	virtual void resetState() override;

	/// Set the initial state (in the global frame)
	/// \param initialState The initial state for this deformable representation (will potentially be changed)
	/// \note 'initialState' will be transformed by the initialPose, if any has been specified
	/// \note The parameter will be kept internally as the shared_ptr and the content will be transformed,
	/// \note   so after this call, do not expect 'initialState' to be unchanged.
	/// \note All internal states are initialized with the transformed initialState to make the simulation ready.
	/// \note This method also sets the number of dof for this Representation
	void setInitialState(std::shared_ptr<DeformableRepresentationState> initialState);

	/// Get the current state (in the global frame)
	/// \return The current state of this deformable representation
	const std::shared_ptr<DeformableRepresentationState> getCurrentState() const;

	/// Get the previous state (in the global frame)
	/// \return The previous state of this deformable representation
	const std::shared_ptr<DeformableRepresentationState> getPreviousState() const;

	/// Get the final state (in the global frame)
	/// \return The final state of this deformable representation
	const std::shared_ptr<DeformableRepresentationState> getFinalState() const;

	/// Gets the number of degrees of freedom per node
	/// \return The number of degrees of freedom per node for this Deformable Representation
	unsigned int getNumDofPerNode() const;

	/// Sets the numerical integration scheme
	/// \param integrationScheme The integration scheme to use
	void setIntegrationScheme(SurgSim::Math::IntegrationScheme integrationScheme);

	/// Gets the numerical integration scheme
	/// \return The integration scheme currently in use
	SurgSim::Math::IntegrationScheme getIntegrationScheme() const;

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	/// \note DeformableRepresentation::beforeUpdate takes care of the OdeSolver setup
	/// \note All derived classes overriding this method should call DeformableRepresentation::beforeUpdate(dt)
	virtual void beforeUpdate(double dt) override;

protected:
	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	virtual void transformState(std::shared_ptr<DeformableRepresentationState> state,
		const SurgSim::Math::RigidTransform3d& transform) = 0;

	/// Previous, current and new states (internal use)
	/// \note New state is a temporary variable to store the newly computed state
	/// \note The initial state is held by the OdeEquation
	std::shared_ptr<DeformableRepresentationState> m_previousState, m_currentState, m_newState;

	/// Last valid state (a.k.a final state)
	/// \note Backup of the current state for thread-safety access while the current state is being recomputed.
	std::shared_ptr<DeformableRepresentationState> m_finalState;

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

	/// Number of degrees of freedom per node (varies per deformable model)
	/// \note MUST be set by the derived classes
	unsigned int m_numDofPerNode;

	/// Numerical Integration scheme (dynamic explicit/implicit solver)
	SurgSim::Math::IntegrationScheme m_integrationScheme;

	/// Specify if the Ode Solver needs to be (re)loaded (do not exist yet, or integration scheme has changed)
	bool m_needToReloadOdeSolver;

	/// Ode solver (its type depends on the numerical integration scheme)
	std::shared_ptr<OdeSolver<DeformableRepresentationState, MType, DType, KType, SType>> m_odeSolver;

private:
	/// NO copy constructor
	DeformableRepresentation(const DeformableRepresentation& a);

	/// NO assignment operator
	DeformableRepresentation& operator =(const DeformableRepresentation& a);

	// Dependent names resolution (need to be in public/protected to be accessible in derived classes)
public:
	using OdeEquation<DeformableRepresentationState, MType, DType, KType, SType>::m_initialState;
};

}; // namespace Physics

}; // namespace SurgSim

#include "SurgSim/Physics/DeformableRepresentation-inl.h"

#endif // SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H

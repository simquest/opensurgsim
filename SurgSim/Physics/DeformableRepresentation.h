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

#include "SurgSim/Physics/DeformableRepresentationBase.h"
#include "SurgSim/Physics/DeformableRepresentationState.h"

#include "SurgSim/Math/OdeEquation.h"
#include "SurgSim/Math/OdeSolver.h"

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
	public DeformableRepresentationBase,
	public SurgSim::Math::OdeEquation<DeformableRepresentationState, MType, DType, KType, SType>
{
public:
	/// Constructor
	/// \param name The deformable representation's name
	explicit DeformableRepresentation(const std::string& name);

	/// Destructor
	virtual ~DeformableRepresentation();

	virtual void driveElement() override;

	virtual void resetState() override;

	virtual void setInitialState(std::shared_ptr<DeformableRepresentationState> initialState) override;

	virtual const std::shared_ptr<DeformableRepresentationState> getCurrentState() const override;

	virtual const std::shared_ptr<DeformableRepresentationState> getPreviousState() const override;

	virtual const std::shared_ptr<DeformableRepresentationState> getFinalState() const override;

	/// Gets the number of degrees of freedom per node
	/// \return The number of degrees of freedom per node for this Deformable Representation
	unsigned int getNumDofPerNode() const;

	/// Sets the numerical integration scheme
	/// \param integrationScheme The integration scheme to use
	void setIntegrationScheme(SurgSim::Math::IntegrationScheme integrationScheme);

	/// Gets the numerical integration scheme
	/// \return The integration scheme currently in use
	SurgSim::Math::IntegrationScheme getIntegrationScheme() const;

	/// Gets the compliance matrix associated with motion
	/// \return The compliance matrix
	const SurgSim::Math::Matrix& getComplianceMatrix() const;

	virtual void beforeUpdate(double dt) override;

	/// Set the collision representation for this physics representation, when the collision object
	/// is involved in a collision, the collision should be resolved inside the dynamics calculation.
	/// Specializes for discarding anything besides a rigid collision representation.
	/// \param representation The collision representation to be used.
	virtual void setCollisionRepresentation(
		std::shared_ptr<SurgSim::Collision::Representation> representation) override;

protected:
	bool doWakeUp() override;

	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	virtual void transformState(std::shared_ptr<DeformableRepresentationState> state,
								const SurgSim::Math::RigidTransform3d& transform) = 0;

	/// The previous state inside the calculation loop, this has no meaning outside of the loop
	std::shared_ptr<DeformableRepresentationState> m_previousState;

	/// The currently calculated state inside the physics loop, after the whole calculation is done this will
	/// become m_finalState
	std::shared_ptr<DeformableRepresentationState> m_currentState;

	/// New state is a temporary variable to store the newly computed state
	std::shared_ptr<DeformableRepresentationState> m_newState;

	/// Last valid state (a.k.a final state)
	/// \note Backup of the current state for thread-safety access while the current state is being recomputed.
	std::shared_ptr<DeformableRepresentationState> m_finalState;

	/// Force applied on the deformable representation
	SurgSim::Math::Vector m_f;

	/// Mass matrix (templated type for performance reason)
	MType m_M;

	/// Damping matrix (templated type for performance reason)
	DType m_D;

	/// Stiffness matrix (templated type for performance reason)
	KType m_K;

	/// Number of degrees of freedom per node (varies per deformable model)
	/// \note MUST be set by the derived classes
	unsigned int m_numDofPerNode;

	/// Numerical Integration scheme (dynamic explicit/implicit solver)
	SurgSim::Math::IntegrationScheme m_integrationScheme;

	/// Specify if the Ode Solver needs to be (re)loaded (do not exist yet, or integration scheme has changed)
	bool m_needToReloadOdeSolver;

	/// Ode solver (its type depends on the numerical integration scheme)
	std::shared_ptr<SurgSim::Math::OdeSolver<DeformableRepresentationState, MType, DType, KType, SType>> m_odeSolver;

private:
	/// NO copy constructor
	DeformableRepresentation(const DeformableRepresentation& a);

	/// NO assignment operator
	DeformableRepresentation& operator =(const DeformableRepresentation& a);

	// Dependent names resolution (need to be in public/protected to be accessible in derived classes)
public:
	using SurgSim::Math::OdeEquation<DeformableRepresentationState, MType, DType, KType, SType>::m_initialState;
};

}; // namespace Physics

}; // namespace SurgSim

#include "SurgSim/Physics/DeformableRepresentation-inl.h"

#endif // SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H




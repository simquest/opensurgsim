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

/// \file DeformableRepresentation.h
/// Base class for all deformable representations (abstract class)

#ifndef SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H
#define SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H

#include <memory>

#include "SurgSim/Math/OdeEquation.h"
#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Physics/Representation.h"

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
/// \note It holds the force vector; the mass, damping and stiffness matrices
/// \note Derived classes must implement the Representation API and the OdeEquation API, also set
/// \note   m_numDofPerNode and call Representation::setNumDof()
class DeformableRepresentation :
	public Representation,
	public SurgSim::Math::OdeEquation
{
public:
	/// Constructor
	/// \param name The deformable representation's name
	explicit DeformableRepresentation(const std::string& name);

	/// Destructor
	virtual ~DeformableRepresentation();

	virtual void resetState() override;

	virtual void setInitialState(std::shared_ptr<SurgSim::Math::OdeState> initialState);

	virtual const std::shared_ptr<SurgSim::Math::OdeState> getCurrentState() const;

	virtual const std::shared_ptr<SurgSim::Math::OdeState> getPreviousState() const;

	virtual const std::shared_ptr<SurgSim::Math::OdeState> getFinalState() const;

	/// Gets the number of degrees of freedom per node
	/// \return The number of degrees of freedom per node for this Deformable Representation
	size_t getNumDofPerNode() const;

	/// Sets the numerical integration scheme
	/// \param integrationScheme The integration scheme to use
	/// \note Calling setIntegrationScheme after the component has been awoken will raise an assert
	void setIntegrationScheme(SurgSim::Math::IntegrationScheme integrationScheme);

	/// Gets the numerical integration scheme
	/// \return The integration scheme currently in use
	SurgSim::Math::IntegrationScheme getIntegrationScheme() const;

	/// Reset all external forces applied on the model
	void resetExternalForce();

	/// Add an external force applied on a specific node of the model
	/// \param nodeId The node id on which the force is applied
	/// \param force The force to apply (of dimension getNumDofPerNode())
	void addExternalForce(size_t nodeId, SurgSim::Math::Vector force);

	/// Gets the compliance matrix associated with motion
	/// \return The compliance matrix
	/// \note The compliance matrix is computed automatically by the ode solver in the method 'update'
	/// \note So one iteration needs to happen before retrieving a compliance matrix
	const SurgSim::Math::Matrix& getComplianceMatrix() const;

	virtual void update(double dt) override;

	virtual void afterUpdate(double dt) override;

	virtual void applyCorrection(double dt, const Eigen::VectorBlock<SurgSim::Math::Vector>& deltaVelocity) override;

	/// Deactivate and call resetState
	void deactivateAndReset(void);

	/// Set the collision representation for this physics representation, when the collision object
	/// is involved in a collision, the collision should be resolved inside the dynamics calculation.
	/// Specializes for discarding anything besides a rigid collision representation.
	/// \param representation The collision representation to be used.
	virtual void setCollisionRepresentation(
		std::shared_ptr<SurgSim::Collision::Representation> representation) override;

protected:
	virtual bool doWakeUp() override;

	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	virtual void transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
								const SurgSim::Math::RigidTransform3d& transform) = 0;

	/// The previous state inside the calculation loop, this has no meaning outside of the loop
	std::shared_ptr<SurgSim::Math::OdeState> m_previousState;

	/// The currently calculated state inside the physics loop, after the whole calculation is done this will
	/// become m_finalState
	std::shared_ptr<SurgSim::Math::OdeState> m_currentState;

	/// New state is a temporary variable to store the newly computed state
	std::shared_ptr<SurgSim::Math::OdeState> m_newState;

	/// Last valid state (a.k.a final state)
	/// \note Backup of the current state for thread-safety access while the current state is being recomputed.
	std::shared_ptr<SurgSim::Math::OdeState> m_finalState;

	/// External force applied on the deformable representation
	SurgSim::Math::Vector m_externalForce;

	/// Force applied on the deformable representation
	SurgSim::Math::Vector m_f;

	/// Mass matrix
	SurgSim::Math::Matrix m_M;

	/// Damping matrix
	SurgSim::Math::Matrix m_D;

	/// Stiffness matrix
	SurgSim::Math::Matrix m_K;

	/// Number of degrees of freedom per node (varies per deformable model)
	/// \note MUST be set by the derived classes
	size_t m_numDofPerNode;

	/// Numerical Integration scheme (dynamic explicit/implicit solver)
	SurgSim::Math::IntegrationScheme m_integrationScheme;

	/// Specify if the Ode Solver needs to be (re)loaded (do not exist yet, or integration scheme has changed)
	bool m_needToReloadOdeSolver;

	/// Ode solver (its type depends on the numerical integration scheme)
	std::shared_ptr<SurgSim::Math::OdeSolver> m_odeSolver;

private:
	/// NO copy constructor
	DeformableRepresentation(const DeformableRepresentation&);

	/// NO assignment operator
	DeformableRepresentation& operator =(const DeformableRepresentation&);
};

}; // namespace Physics

}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_H




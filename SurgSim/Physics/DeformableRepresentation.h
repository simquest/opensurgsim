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

#include "SurgSim/Math/LinearSparseSolveAndInverse.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeEquation.h"
#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{

namespace Physics
{

class Localization;

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

	void resetState() override;

	/// Initialize the state variables to initialState
	/// \param initialState is the state to be set as the starting state
	virtual void setInitialState(std::shared_ptr<SurgSim::Math::OdeState> initialState);

	/// Return the current state of the deformable representation
	/// \return the current state
	virtual const std::shared_ptr<SurgSim::Math::OdeState> getCurrentState() const;

	/// Return the previous state of the deformable representation
	/// \return the previous state
	virtual const std::shared_ptr<SurgSim::Math::OdeState> getPreviousState() const;

	/// Return the final state of the deformable representation
	/// \return the final state
	virtual const std::shared_ptr<SurgSim::Math::OdeState> getFinalState() const;

	/// Declare a new previous state by interpolating between the old previous
	/// state and the current state using parametric time variable t
	/// \param t parametric time at which to calculate the new state
	virtual void interpolatePreviousState(double t);

	/// Gets the number of degrees of freedom per node
	/// \return The number of degrees of freedom per node for this Deformable Representation
	size_t getNumDofPerNode() const;

	/// Sets the numerical integration scheme
	/// \param integrationScheme The integration scheme to use
	/// \exception SurgSim::Framework::AssertionFailure raised if called after the component has been initialized.
	void setIntegrationScheme(SurgSim::Math::IntegrationScheme integrationScheme);

	/// Gets the numerical integration scheme
	/// \return The integration scheme currently in use
	/// \note Default is SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT
	SurgSim::Math::IntegrationScheme getIntegrationScheme() const;

	/// \return The ode solver (dependent on the integration scheme)
	/// \note Will return nullptr if called before initialization.
	std::shared_ptr<SurgSim::Math::OdeSolver> getOdeSolver() const;

	/// Sets the linear algebraic solver
	/// \param linearSolver The linear algebraic solver to use
	/// \exception SurgSim::Framework::AssertionFailure raised if called after the component has been initialized.
	void setLinearSolver(SurgSim::Math::LinearSolver linearSolver);

	/// Gets the linear algebraic solver
	/// \return The linear solver currently in use
	/// \note Default is SurgSim::Math::LINEARSOLVER_LU
	SurgSim::Math::LinearSolver getLinearSolver() const;

	/// Add an external generalized force applied on a specific localization
	/// \param localization where the generalized force is applied
	/// \param generalizedForce The force to apply (of dimension getNumDofPerNode())
	/// \param K The stiffness matrix associated with the generalized force (Jacobian of the force w.r.t dof's position)
	/// \param D The damping matrix associated with the generalized force (Jacobian of the force w.r.t dof's velocity)
	virtual void addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
			const SurgSim::Math::Vector& generalizedForce,
			const SurgSim::Math::Matrix& K = SurgSim::Math::Matrix(),
			const SurgSim::Math::Matrix& D = SurgSim::Math::Matrix()) = 0;

	/// \return the external generalized force vector
	const SurgSim::Math::Vector& getExternalGeneralizedForce() const;

	/// \return the external generalized stiffness matrix
	const SurgSim::Math::SparseMatrix& getExternalGeneralizedStiffness() const;

	/// \return the external generalized damping matrix
	const SurgSim::Math::SparseMatrix& getExternalGeneralizedDamping() const;

	Math::Matrix applyCompliance(const Math::OdeState& state, const Math::Matrix& b) override;

	/// Gets the compliance matrix associated with motion
	virtual const SurgSim::Math::Matrix& getComplianceMatrix() const;

	void update(double dt) override;

	void afterUpdate(double dt) override;

	void applyCorrection(double dt, const Eigen::VectorBlock<SurgSim::Math::Vector>& deltaVelocity) override;

	/// Deactivate and call resetState
	void deactivateAndReset();

	/// Set the collision representation for this physics representation, when the collision object
	/// is involved in a collision, the collision should be resolved inside the dynamics calculation.
	/// Specializes for discarding anything besides a rigid collision representation.
	/// \param representation The collision representation to be used.
	void setCollisionRepresentation(std::shared_ptr<SurgSim::Collision::Representation> representation) override;

	void setLocalPose(const SurgSim::Math::RigidTransform3d& pose) override;



protected:
	bool doInitialize() override;
	bool doWakeUp() override;

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

	/// External generalized force, stiffness and damping applied on the deformable representation
	/// @{
	bool m_hasExternalGeneralizedForce;
	SurgSim::Math::Vector m_externalGeneralizedForce;
	SurgSim::Math::SparseMatrix m_externalGeneralizedStiffness;
	SurgSim::Math::SparseMatrix m_externalGeneralizedDamping;
	bool m_previousHasExternalGeneralizedForce;
	SurgSim::Math::SparseMatrix m_previousExternalGeneralizedStiffness;
	SurgSim::Math::SparseMatrix m_previousExternalGeneralizedDamping;
	/// @}

	/// Number of degrees of freedom per node (varies per deformable model)
	/// \note MUST be set by the derived classes
	size_t m_numDofPerNode;

	/// Numerical Integration scheme (dynamic explicit/implicit solver)
	SurgSim::Math::IntegrationScheme m_integrationScheme;

	/// Linear algebraic solver used
	SurgSim::Math::LinearSolver m_linearSolver;

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




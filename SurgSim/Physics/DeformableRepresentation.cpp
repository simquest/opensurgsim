// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Math/LinearSparseSolveAndInverse.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/OdeSolverEulerExplicit.h"
#include "SurgSim/Math/OdeSolverEulerExplicitModified.h"
#include "SurgSim/Math/OdeSolverEulerImplicit.h"
#include "SurgSim/Math/OdeSolverRungeKutta4.h"
#include "SurgSim/Math/OdeSolverLinearEulerExplicit.h"
#include "SurgSim/Math/OdeSolverLinearEulerExplicitModified.h"
#include "SurgSim/Math/OdeSolverLinearEulerImplicit.h"
#include "SurgSim/Math/OdeSolverLinearRungeKutta4.h"
#include "SurgSim/Math/OdeSolverLinearStatic.h"
#include "SurgSim/Math/OdeSolverStatic.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/DeformableRepresentation.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::OdeState;
using SurgSim::Math::SparseMatrix;
using SurgSim::Math::Vector;

namespace SurgSim
{

namespace Physics
{

DeformableRepresentation::DeformableRepresentation(const std::string& name) :
	Representation(name),
	SurgSim::Math::OdeEquation(),
	m_numDofPerNode(0),
	m_integrationScheme(SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER),
	m_linearSolver(SurgSim::Math::LINEARSOLVER_LU)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(DeformableRepresentation, SurgSim::Math::IntegrationScheme, IntegrationScheme,
									  getIntegrationScheme, setIntegrationScheme);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(DeformableRepresentation, SurgSim::Math::LinearSolver, LinearSolver,
									  getLinearSolver, setLinearSolver);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(DeformableRepresentation, std::shared_ptr<SurgSim::Collision::Representation>,
									  CollisionRepresentation, getCollisionRepresentation, setCollisionRepresentation);
}

DeformableRepresentation::~DeformableRepresentation()
{
}

void DeformableRepresentation::resetState()
{
	Representation::resetState();

	// Reminder: m_initialState is being held in OdeEquation
	*m_currentState  = *m_initialState;
	*m_previousState = *m_initialState;
	// m_newState does not need to be reset, it is a temporary variable
	*m_finalState    = *m_initialState;
}

void DeformableRepresentation::setInitialState(
	std::shared_ptr<SurgSim::Math::OdeState> initialState)
{
	// This initializes and allocates the m_initialState data member
	m_initialState = initialState;

	m_previousState = std::make_shared<SurgSim::Math::OdeState>(*m_initialState);
	m_currentState = std::make_shared<SurgSim::Math::OdeState>(*m_initialState);
	m_newState = std::make_shared<SurgSim::Math::OdeState>(*m_initialState);
	m_finalState = std::make_shared<SurgSim::Math::OdeState>(*m_initialState);

	// Set the representation number of degree of freedom
	setNumDof(m_initialState->getNumDof());

	m_hasExternalGeneralizedForce = false;
	m_externalGeneralizedForce.setZero(getNumDof());
	m_externalGeneralizedStiffness.resize(static_cast<SparseMatrix::Index>(getNumDof()),
										  static_cast<SparseMatrix::Index>(getNumDof()));
	m_externalGeneralizedDamping.resize(static_cast<SparseMatrix::Index>(getNumDof()),
										static_cast<SparseMatrix::Index>(getNumDof()));
}

const std::shared_ptr<SurgSim::Math::OdeState> DeformableRepresentation::getCurrentState() const
{
	return m_currentState;
}

const std::shared_ptr<SurgSim::Math::OdeState> DeformableRepresentation::getPreviousState() const
{
	return m_previousState;
}

const std::shared_ptr<SurgSim::Math::OdeState> DeformableRepresentation::getFinalState() const
{
	return m_finalState;
}

size_t DeformableRepresentation::getNumDofPerNode() const
{
	return m_numDofPerNode;
}

void DeformableRepresentation::setIntegrationScheme(SurgSim::Math::IntegrationScheme integrationScheme)
{
	SURGSIM_ASSERT(!isAwake()) << "You cannot set the integration scheme after the component has been awoken";
	m_integrationScheme = integrationScheme;
}

SurgSim::Math::IntegrationScheme DeformableRepresentation::getIntegrationScheme() const
{
	return m_integrationScheme;
}

void DeformableRepresentation::setLinearSolver(SurgSim::Math::LinearSolver linearSolver)
{
	SURGSIM_ASSERT(!isAwake()) << "You cannot set the linear solver after the component has been awoken";
	m_linearSolver = linearSolver;
}

SurgSim::Math::LinearSolver DeformableRepresentation::getLinearSolver() const
{
	return m_linearSolver;
}

const SurgSim::Math::Vector& DeformableRepresentation::getExternalGeneralizedForce() const
{
	return m_externalGeneralizedForce;
}

const SurgSim::Math::SparseMatrix& DeformableRepresentation::getExternalGeneralizedStiffness() const
{
	return m_externalGeneralizedStiffness;
}

const SurgSim::Math::SparseMatrix& DeformableRepresentation::getExternalGeneralizedDamping() const
{
	return m_externalGeneralizedDamping;
}

Math::Matrix DeformableRepresentation::applyCompliance(const Math::OdeState& state, const Math::Matrix& b)
{
	SURGSIM_ASSERT(m_odeSolver) << "Ode solver not initialized, it should have been initialized on wake-up";

	Math::Matrix bTemp = b;
	for (auto condition : state.getBoundaryConditions())
	{
		Math::zeroRow(condition, &bTemp);
	}
	auto solution = m_odeSolver->getLinearSolver()->solve(bTemp);
	for (auto condition : state.getBoundaryConditions())
	{
		Math::zeroRow(condition, &solution);
	}
	return solution;
}

const SurgSim::Math::Matrix& DeformableRepresentation::getComplianceMatrix() const
{
	SURGSIM_ASSERT(m_odeSolver) << "Ode solver not initialized, it should have been initialized on wake-up";
	return m_odeSolver->getComplianceMatrix();
}

void DeformableRepresentation::update(double dt)
{
	if (! isActive())
	{
		return;
	}

	SURGSIM_ASSERT(m_odeSolver != nullptr) <<
										   "Ode solver has not been set yet. Did you call beforeUpdate() ?";
	SURGSIM_ASSERT(m_initialState != nullptr) <<
			"Initial state has not been set yet. Did you call setInitialState() ?";

	// Solve the ode
	m_odeSolver->solve(dt, *m_currentState, m_newState.get());

	// Back up the current state into the previous state (by swapping)
	m_currentState.swap(m_previousState);
	// Make the new state, the current state (by swapping)
	m_currentState.swap(m_newState);

	if (!m_currentState->isValid())
	{
		SURGSIM_LOG(SurgSim::Framework::Logger::getDefaultLogger(), DEBUG)
				<< getName() << " deactivated :" << std::endl
				<< "position=(" << m_currentState->getPositions().transpose() << ")" << std::endl
				<< "velocity=(" << m_currentState->getVelocities().transpose() << ")" << std::endl;

		setLocalActive(false);
	}
}

void DeformableRepresentation::afterUpdate(double dt)
{
	if (! isActive())
	{
		return;
	}

	driveSceneElementPose(SurgSim::Math::RigidTransform3d::Identity());

	// Back up the current state into the final state
	// #threadsafety
	*m_finalState = *m_currentState;

	// Reset the external generalized force, stiffness and damping
	if (m_hasExternalGeneralizedForce)
	{
		m_externalGeneralizedForce.setZero();
		m_externalGeneralizedStiffness.setZero();
		m_externalGeneralizedDamping.setZero();
		m_hasExternalGeneralizedForce = false;
	}
}

void DeformableRepresentation::applyCorrection(double dt,
		const Eigen::VectorBlock<SurgSim::Math::Vector>& deltaVelocity)
{
	if (!isActive())
	{
		return;
	}

	m_currentState->getPositions() += deltaVelocity * dt;
	m_currentState->getVelocities() += deltaVelocity;

	if (!m_currentState->isValid())
	{
		SURGSIM_LOG(SurgSim::Framework::Logger::getDefaultLogger(), DEBUG)
				<< getName() << " deactivated :" << std::endl
				<< "position=(" << m_currentState->getPositions() << ")" << std::endl
				<< "velocity=(" << m_currentState->getVelocities() << ")" << std::endl;

		setLocalActive(false);
	}
}

void DeformableRepresentation::deactivateAndReset()
{
	SURGSIM_LOG(SurgSim::Framework::Logger::getDefaultLogger(), DEBUG)
			<< getName() << " deactivated and reset:" << std::endl
			<< "position=(" << m_currentState->getPositions() << ")" << std::endl
			<< "velocity=(" << m_currentState->getVelocities() << ")" << std::endl;

	resetState();
	setLocalActive(false);
}

void DeformableRepresentation::setCollisionRepresentation(
	std::shared_ptr<SurgSim::Collision::Representation> representation)
{
	if (m_collisionRepresentation != representation)
	{
		// If we have an old collision representation clear the dependency if it was a deformable collision
		// representation
		auto oldCollisionRep =
			std::dynamic_pointer_cast<DeformableCollisionRepresentation>(m_collisionRepresentation);
		if (oldCollisionRep != nullptr)
		{
			oldCollisionRep->setDeformableRepresentation(nullptr);
		}

		Representation::setCollisionRepresentation(representation);

		// If its a RigidCollisionRepresentation connect with this representation
		auto newCollisionRep = std::dynamic_pointer_cast<DeformableCollisionRepresentation>(representation);
		if (newCollisionRep != nullptr)
		{
			newCollisionRep->setDeformableRepresentation(
				std::static_pointer_cast<DeformableRepresentation>(getSharedPtr()));
		}
	}
}

bool DeformableRepresentation::doWakeUp()
{
	using SurgSim::Math::OdeSolverEulerExplicit;
	using SurgSim::Math::OdeSolverEulerExplicitModified;
	using SurgSim::Math::OdeSolverEulerImplicit;
	using SurgSim::Math::OdeSolverRungeKutta4;
	using SurgSim::Math::OdeSolverStatic;
	using SurgSim::Math::OdeSolverLinearEulerExplicit;
	using SurgSim::Math::OdeSolverLinearEulerExplicitModified;
	using SurgSim::Math::OdeSolverLinearEulerImplicit;
	using SurgSim::Math::OdeSolverLinearRungeKutta4;
	using SurgSim::Math::OdeSolverLinearStatic;

	using SurgSim::Math::LinearSparseSolveAndInverseCG;
	using SurgSim::Math::LinearSparseSolveAndInverseLU;

	// Set the ode solver using the chosen integration scheme
	switch (m_integrationScheme)
	{
		case SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER:
			m_odeSolver = std::make_shared<OdeSolverEulerExplicit>(this);
			break;
		case SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER:
			m_odeSolver = std::make_shared<OdeSolverEulerExplicitModified>(this);
			break;
		case SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER:
			m_odeSolver = std::make_shared<OdeSolverEulerImplicit>(this);
			break;
		case SurgSim::Math::INTEGRATIONSCHEME_STATIC:
			m_odeSolver = std::make_shared<OdeSolverStatic>(this);
			break;
		case SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4:
			m_odeSolver = std::make_shared<OdeSolverRungeKutta4>(this);
			break;
		case SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER:
			m_odeSolver = std::make_shared<OdeSolverLinearEulerExplicit>(this);
			break;
		case SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER:
			m_odeSolver = std::make_shared<OdeSolverLinearEulerExplicitModified>(this);
			break;
		case SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER:
			m_odeSolver = std::make_shared<OdeSolverLinearEulerImplicit>(this);
			break;
		case SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC:
			m_odeSolver = std::make_shared<OdeSolverLinearStatic>(this);
			break;
		case SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4:
			m_odeSolver = std::make_shared<OdeSolverLinearRungeKutta4>(this);
			break;
		default:
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
					<< "Ode solver (integration scheme) not initialized, the integration scheme is invalid";
			return false;
	}


	// Set the linear solver with initial settings on the ode solver
	switch (m_linearSolver)
	{
		case SurgSim::Math::LINEARSOLVER_LU:
			m_odeSolver->setLinearSolver(std::make_shared<LinearSparseSolveAndInverseLU>());
			break;
		case SurgSim::Math::LINEARSOLVER_CONJUGATEGRADIENT:
			m_odeSolver->setLinearSolver(std::make_shared<LinearSparseSolveAndInverseCG>());
			break;
		default:
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
					<< "Linear solver not initialized, the linear solver is invalid";
			return false;
	}

	return true;
}

}; // namespace Physics

}; // namespace SurgSim

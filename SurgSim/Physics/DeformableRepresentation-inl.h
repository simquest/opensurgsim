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

#ifndef SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_INL_H
#define SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_INL_H

#include "SurgSim/Framework/Assert.h"

#include "SurgSim/Math/OdeSolverEulerExplicit.h"
#include "SurgSim/Math/OdeSolverEulerExplicitModified.h"
#include "SurgSim/Math/OdeSolverEulerImplicit.h"

namespace SurgSim
{

namespace Physics
{

template <class M, class D, class K, class S>
DeformableRepresentation<M,D,K,S>::DeformableRepresentation(const std::string& name) :
	Representation(name),
	SurgSim::Math::OdeEquation<DeformableRepresentationState, M, D, K, S>(),
	m_numDofPerNode(0),
	m_integrationScheme(SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER),
	m_needToReloadOdeSolver(true)
{
	m_initialPose.setIdentity();
	m_identityPose.setIdentity();
}

template <class M, class D, class K, class S>
DeformableRepresentation<M,D,K,S>::~DeformableRepresentation()
{
}

template <class M, class D, class K, class S>
void DeformableRepresentation<M,D,K,S>::setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_initialPose = pose;
}

template <class M, class D, class K, class S>
const SurgSim::Math::RigidTransform3d& DeformableRepresentation<M,D,K,S>::getInitialPose() const
{
	return m_initialPose;
}

template <class M, class D, class K, class S>
void DeformableRepresentation<M,D,K,S>::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	SURGSIM_ASSERT(false) << "setPose has been called on a physics DeformableRepresentation";
}

template <class M, class D, class K, class S>
const SurgSim::Math::RigidTransform3d& DeformableRepresentation<M,D,K,S>::getPose() const
{
	return m_identityPose;
}

template <class M, class D, class K, class S>
void DeformableRepresentation<M,D,K,S>::resetState()
{
	Representation::resetState();

	// Reminder: m_initialState is being held in OdeEquation
	*m_currentState  = *m_initialState;
	*m_previousState = *m_initialState;
	// m_newState does not need to be reset, it is a temporary variable
	*m_finalState    = *m_initialState;
}

template <class M, class D, class K, class S>
void DeformableRepresentation<M,D,K,S>::setInitialState(std::shared_ptr<DeformableRepresentationState> initialState)
{
	// This initializes and allocates the m_initialState data member
	m_initialState = initialState;
	transformState(m_initialState, m_initialPose);

	m_previousState = std::make_shared<DeformableRepresentationState>(*m_initialState);
	m_currentState = std::make_shared<DeformableRepresentationState>(*m_initialState);
	m_newState = std::make_shared<DeformableRepresentationState>(*m_initialState);
	m_finalState = std::make_shared<DeformableRepresentationState>(*m_initialState);

	// Set the representation number of degree of freedom
	setNumDof(m_initialState->getNumDof());
}

template <class M, class D, class K, class S>
const std::shared_ptr<DeformableRepresentationState> DeformableRepresentation<M,D,K,S>::getCurrentState() const
{
	return m_currentState;
}

template <class M, class D, class K, class S>
const std::shared_ptr<DeformableRepresentationState> DeformableRepresentation<M,D,K,S>::getPreviousState() const
{
	return m_previousState;
}

template <class M, class D, class K, class S>
const std::shared_ptr<DeformableRepresentationState> DeformableRepresentation<M,D,K,S>::getFinalState() const
{
	return m_finalState;
}

template <class M, class D, class K, class S>
unsigned int DeformableRepresentation<M,D,K,S>::getNumDofPerNode() const
{
	return m_numDofPerNode;
}

template <class M, class D, class K, class S>
void DeformableRepresentation<M,D,K,S>::setIntegrationScheme(SurgSim::Math::IntegrationScheme integrationScheme)
{
	if (m_integrationScheme != integrationScheme )
	{
		// Sets the new integration scheme
		m_integrationScheme = integrationScheme;
		// The integration scheme has changed, the ode solver needs to be reloaded
		m_needToReloadOdeSolver = true;
	}
}

template <class M, class D, class K, class S>
SurgSim::Math::IntegrationScheme DeformableRepresentation<M,D,K,S>::getIntegrationScheme() const
{
	return m_integrationScheme;
}

template <class M, class D, class K, class S>
const SurgSim::Math::Matrix& DeformableRepresentation<M,D,K,S>::getComplianceMatrix() const
{
	SURGSIM_ASSERT(m_odeSolver) <<
		"Ode solver not initialized yet, call beforeUpdate(dt)";

	return m_odeSolver->getCompliance();
}

template <class M, class D, class K, class S>
void  DeformableRepresentation<M,D,K,S>::beforeUpdate(double dt)
{
	using SurgSim::Math::ExplicitEuler;
	using SurgSim::Math::ModifiedExplicitEuler;
	using SurgSim::Math::ImplicitEuler;

	if (! isActive())
	{
		return;
	}

	if (m_needToReloadOdeSolver)
	{
		if (m_odeSolver)
		{
			// If the ode solver exist already, we need to reset it first (free memory)
			m_odeSolver.reset();
		}

		switch(m_integrationScheme)
		{
		case SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER:
			m_odeSolver = std::make_shared
				<ExplicitEuler<DeformableRepresentationState, M, D, K, S>>(this);
			break;
		case SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER:
			m_odeSolver = std::make_shared
				<ModifiedExplicitEuler<DeformableRepresentationState, M, D, K, S>>(this);
			break;
		case SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER:
			m_odeSolver = std::make_shared
				<ImplicitEuler<DeformableRepresentationState, M, D, K, S>>(this);
			break;
		default:
			SURGSIM_ASSERT(m_odeSolver) <<
				"Ode solver (integration scheme) not initialized yet, call setIntegrationScheme()";
			break;
		}

		m_needToReloadOdeSolver = false;
	}
}

}; // namespace Physics

}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_INL_H

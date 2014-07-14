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

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationBase.h"
#include "SurgSim/Physics/RigidRepresentationLocalization.h"
#include "SurgSim/Physics/PhysicsConvert.h"

namespace SurgSim
{
namespace Physics
{

RigidRepresentationBase::RigidRepresentationBase(const std::string& name) : Representation(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationBase, RigidRepresentationState,
		RigidRepresentationState, getInitialState, setInitialState);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationBase, RigidRepresentationParameters,
		RigidRepresentationParameters, getInitialParameters, setInitialParameters);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationBase, std::shared_ptr<SurgSim::Collision::Representation>,
		CollisionRepresentation, getCollisionRepresentation, setCollisionRepresentation);
}

RigidRepresentationBase::~RigidRepresentationBase()
{
}

bool RigidRepresentationBase::doInitialize()
{
	auto meshShape =
		std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(getInitialParameters().getShapeUsedForMassInertia());
	if (nullptr != meshShape)
	{
		SURGSIM_ASSERT(meshShape->isValid()) <<
			"An invalid MeshShape is used in this RigidRepresentationBase.";

		m_initialParameters.updateProperties();
		setCurrentParameters(m_initialParameters);
	}

	return true;
}

bool RigidRepresentationBase::doWakeUp()
{
	m_initialState.setPose(getPose());
	m_currentState = m_initialState;
	m_finalState = m_initialState;
	m_previousState = m_initialState;
	updateGlobalInertiaMatrices(m_currentState);
	return true;
}

void RigidRepresentationBase::setInitialState(const RigidRepresentationState& state)
{
	m_initialState = state;
	m_currentState = state;
	m_previousState = state;

	updateGlobalInertiaMatrices(m_currentState);
}

void RigidRepresentationBase::resetState()
{
	Representation::resetState();

	m_currentState  = m_initialState;
	m_previousState = m_initialState;
	m_finalState    = m_initialState;

	updateGlobalInertiaMatrices(m_currentState);
}

const RigidRepresentationState& RigidRepresentationBase::getInitialState() const
{
	return m_initialState;
}

const RigidRepresentationState& RigidRepresentationBase::getCurrentState() const
{
	return m_currentState;
}

const RigidRepresentationState& RigidRepresentationBase::getPreviousState() const
{
	return m_previousState;
}

std::shared_ptr<Localization> RigidRepresentationBase::createLocalization(const SurgSim::Collision::Location& location)
{
	return std::move(createTypedLocalization<RigidRepresentationLocalization>(location));
}

void RigidRepresentationBase::setInitialParameters(const RigidRepresentationParameters& parameters)
{
	m_initialParameters = parameters;
	m_currentParameters = parameters;

	updateGlobalInertiaMatrices(m_currentState);
}

void RigidRepresentationBase::setCurrentParameters(const RigidRepresentationParameters& parameters)
{
	m_currentParameters = parameters;
	updateGlobalInertiaMatrices(m_currentState);
}

const RigidRepresentationParameters& SurgSim::Physics::RigidRepresentationBase::getInitialParameters() const
{
	return m_initialParameters;
}

const RigidRepresentationParameters& SurgSim::Physics::RigidRepresentationBase::getCurrentParameters() const
{
	return m_currentParameters;
}

void SurgSim::Physics::RigidRepresentationBase::beforeUpdate(double dt)
{
	m_previousState = m_currentState;
}

void SurgSim::Physics::RigidRepresentationBase::afterUpdate(double dt)
{
	m_finalState = m_currentState;
	if (isDrivingSceneElementPose())
	{
		std::shared_ptr<SurgSim::Framework::PoseComponent> poseComponent = getPoseComponent();
		if (poseComponent != nullptr)
		{
			poseComponent->setPose(m_finalState.getPose() * getLocalPose().inverse());
		}
	}
}

void RigidRepresentationBase::setCollisionRepresentation(
	std::shared_ptr<SurgSim::Collision::Representation> representation)
{
	if (m_collisionRepresentation != representation)
	{
		// If we have an old collision representation clear the dependency
		auto oldCollisionRep = std::dynamic_pointer_cast<RigidCollisionRepresentation>(m_collisionRepresentation);
		if (oldCollisionRep != nullptr)
		{
			oldCollisionRep->setRigidRepresentation(nullptr);
		}

		Representation::setCollisionRepresentation(representation);

		// If its a RigidCollisionRepresentation connect with this representation
		auto newCollisionRep = std::dynamic_pointer_cast<RigidCollisionRepresentation>(representation);
		if (newCollisionRep != nullptr)
		{
			newCollisionRep->setRigidRepresentation(std::static_pointer_cast<RigidRepresentationBase>(getSharedPtr()));
		}
	}
}

}; // Physics
}; // SurgSim

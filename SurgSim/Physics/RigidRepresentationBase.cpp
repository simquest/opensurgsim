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
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/Valid.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationBase.h"
#include "SurgSim/Physics/PhysicsConvert.h"

namespace SurgSim
{
namespace Physics
{

RigidRepresentationBase::RigidRepresentationBase(const std::string& name) :
	Representation(name),
	m_parametersValid(false),
	m_rho(0.0),
	m_mass(std::numeric_limits<double>::quiet_NaN()),
	m_linearDamping(0.0),
	m_angularDamping(0.0)
{
	m_localInertia.setConstant(std::numeric_limits<double>::quiet_NaN());
	m_massCenter.setConstant(std::numeric_limits<double>::quiet_NaN());

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationBase, RigidRepresentationState,
									  RigidRepresentationState, getInitialState, setInitialState);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationBase, std::shared_ptr<SurgSim::Collision::Representation>,
									  CollisionRepresentation, getCollisionRepresentation, setCollisionRepresentation);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationBase, double, Density, getDensity, setDensity);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationBase, double, LinearDamping,
									  getLinearDamping, setLinearDamping);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationBase, double, AngularDamping,
									  getAngularDamping, setAngularDamping);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationBase, std::shared_ptr<SurgSim::Math::Shape>, Shape,
									  getShape, setShape);

}

RigidRepresentationBase::~RigidRepresentationBase()
{
}

bool RigidRepresentationBase::doInitialize()
{
	if (m_shape != nullptr)
	{
		SURGSIM_ASSERT(m_shape->isValid()) <<
			"An invalid shape is used in this RigidRepresentationBase.";
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

std::shared_ptr<Localization> RigidRepresentationBase::createLocalization(
	const SurgSim::DataStructures::Location& location)
{
	return std::move(createTypedLocalization<RigidRepresentationBaseLocalization>(location));
}

void RigidRepresentationBase::setDensity(double rho)
{
	m_rho = rho;
	updateProperties();
}

double RigidRepresentationBase::getDensity() const
{
	return m_rho;
}

double RigidRepresentationBase::getMass() const
{
	return m_mass;
}

const SurgSim::Math::Vector3d& RigidRepresentationBase::getMassCenter() const
{
	return m_massCenter;
}

const SurgSim::Math::Matrix33d& RigidRepresentationBase::getLocalInertia() const
{
	return m_localInertia;
}

void RigidRepresentationBase::setLinearDamping(double linearDamping)
{
	m_linearDamping = linearDamping;
}

double RigidRepresentationBase::getLinearDamping() const
{
	return m_linearDamping;
}

void RigidRepresentationBase::setAngularDamping(double angularDamping)
{
	m_angularDamping = angularDamping;
}

double RigidRepresentationBase::getAngularDamping() const
{
	return m_angularDamping;
}

void RigidRepresentationBase::setShape(const std::shared_ptr<SurgSim::Math::Shape> shape)
{
	m_shape = shape;
	if (shape != nullptr)
	{
		updateProperties();
	}
}

const std::shared_ptr<SurgSim::Math::Shape> RigidRepresentationBase::getShape() const
{
	return m_shape;
}

void RigidRepresentationBase::updateProperties()
{
	if (m_shape != nullptr)
	{
		SURGSIM_ASSERT(m_shape->isValid()) << "Invalid shape.";
		m_massCenter   = m_shape->getCenter();
		if (m_rho > 0.0)
		{
			m_mass         = m_rho * m_shape->getVolume();
			m_localInertia = m_rho * m_shape->getSecondMomentOfVolume();
			m_parametersValid = SurgSim::Math::isValid(m_localInertia) &&
				!m_localInertia.isZero() &&
				m_localInertia.diagonal().minCoeff() > 0.0 &&
				SurgSim::Math::isValid(m_mass) && m_mass > 0.0;
		}
	}
}

void SurgSim::Physics::RigidRepresentationBase::beforeUpdate(double dt)
{
	m_previousState = m_currentState;
}

void SurgSim::Physics::RigidRepresentationBase::afterUpdate(double dt)
{
	m_finalState = m_currentState;
	driveSceneElementPose(m_finalState.getPose() * getLocalPose().inverse());
}

Math::Vector3d RigidRepresentationBase::getVelocityAt(std::shared_ptr<Localization> localization)
{
	Math::Vector3d velocity(0.0, 0.0, 0.0);
	auto rigidBaseLocalization = std::dynamic_pointer_cast<RigidRepresentationBaseLocalization>(localization);
	if (rigidBaseLocalization != nullptr)
	{
		Math::Vector3d localPosition = rigidBaseLocalization->getLocalPosition() - getMassCenter();
		velocity = m_currentState.getLinearVelocity() + m_currentState.getAngularVelocity().cross(localPosition);
	}
	return velocity;
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

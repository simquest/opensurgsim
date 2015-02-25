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

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Physics
{

Representation::Representation(const std::string& name) :
	SurgSim::Framework::Representation(name),
	m_collisionRepresentation(nullptr),
	m_gravity(0.0, -9.81, 0.0),
	m_numDof(0),
	m_isGravityEnabled(true),
	m_isDrivingSceneElementPose(true),
	m_logger(SurgSim::Framework::Logger::getLogger("Physics/Representation"))
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, size_t, NumDof, getNumDof, setNumDof);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, bool, IsGravityEnabled, isGravityEnabled, setIsGravityEnabled);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, bool, IsDrivingSceneElementPose,
									  isDrivingSceneElementPose, setIsDrivingSceneElementPose);
}

Representation::~Representation()
{
}

void Representation::resetState()
{
}

void Representation::resetParameters()
{
}

size_t Representation::getNumDof() const
{
	return m_numDof;
}

void Representation::setIsGravityEnabled(bool isGravityEnabled)
{
	m_isGravityEnabled = isGravityEnabled;
}

bool Representation::isGravityEnabled() const
{
	return m_isGravityEnabled;
}

void Representation::setIsDrivingSceneElementPose(bool isDrivingSceneElementPose)
{
	m_isDrivingSceneElementPose = isDrivingSceneElementPose;
}

bool Representation::isDrivingSceneElementPose()
{
	return m_isDrivingSceneElementPose;
}

void Representation::beforeUpdate(double dt)
{
}

void Representation::update(double dt)
{
}

void Representation::afterUpdate(double dt)
{
}

std::shared_ptr<Localization> Representation::createLocalization(const SurgSim::DataStructures::Location& location)
{
	return nullptr;
}

std::shared_ptr<Constraint> Representation::createConstraint(
	SurgSim::Math::MlcpConstraintType type,
	std::shared_ptr<Localization> thisLocalization,
	std::shared_ptr<Localization> otherLocalization,
	std::shared_ptr<ConstraintData> constraintData)
{
	if (thisLocalization == nullptr || otherLocalization == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << getClassName() << ": Constraint cannot be created without localizations.";
		return nullptr;
	}

	if (thisLocalization->getRepresentation() == nullptr || thisLocalization->getRepresentation().get() != this)
	{
		SURGSIM_LOG_SEVERE(m_logger) << getClassName() <<
			": thisLocalization has to have a representation attached to it, and the representation" <<
			"should be the object on which this createConstraint() is called.";
		return nullptr;
	}

	if (otherLocalization->getRepresentation() == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << getClassName() <<
			": otherLocalization has to have a representation attached to it.";
		return nullptr;
	}

	if (otherLocalization.get() == thisLocalization.get())
	{
		SURGSIM_LOG_SEVERE(m_logger) << getClassName() <<
			": otherLocalization has to be different from thisLocalization.";
		return nullptr;
	}

	auto thisImplementation = thisLocalization->getRepresentation()->getConstraintImplementation(type);
	auto otherImplementation = otherLocalization->getRepresentation()->getConstraintImplementation(type);
	if (thisImplementation != nullptr && otherImplementation != nullptr)
	{
		if (constraintData == nullptr)
		{
			constraintData = std::make_shared<ConstraintData>();
		}
		return std::make_shared<Constraint>(
			constraintData,
			thisImplementation,
			thisLocalization,
			otherImplementation,
			otherLocalization);
	}

	return nullptr;
}

void Representation::applyCorrection(double dt, const Eigen::VectorBlock<SurgSim::Math::Vector>& deltaVelocity)
{
}

void Representation::setNumDof(size_t numDof)
{
	m_numDof = numDof;
}

const SurgSim::Math::Vector3d& Representation::getGravity() const
{
	return m_gravity;
}

std::shared_ptr<SurgSim::Collision::Representation> Representation::getCollisionRepresentation() const
{
	return m_collisionRepresentation;
}

void Representation::setCollisionRepresentation(std::shared_ptr<SurgSim::Collision::Representation> val)
{
	m_collisionRepresentation = val;
}

void Representation::driveSceneElementPose(const SurgSim::Math::RigidTransform3d& pose)
{
	if (isDrivingSceneElementPose())
	{
		std::shared_ptr<SurgSim::Framework::SceneElement> sceneElement = getSceneElement();
		if (sceneElement != nullptr)
		{
			sceneElement->setPose(pose);
		}
	}
}

std::shared_ptr<ConstraintImplementation> Representation::getConstraintImplementation(
	SurgSim::Math::MlcpConstraintType type)
{
	auto implementation = ConstraintImplementation::getFactory().getImplementation(typeid(*this), type);
	if (implementation == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << getClassName() << ": Does not support constraint type (" << type << ").";
	}
	return implementation;
}

}; // namespace Physics
}; // namespace SurgSim

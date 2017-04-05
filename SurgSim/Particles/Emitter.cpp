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

#include "SurgSim/Particles/Emitter.h"

#include <utility>

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Particles/Representation.h"

using SurgSim::Math::Vector3d;


namespace SurgSim
{

namespace Particles
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Particles::Emitter, Emitter);

Emitter::Emitter(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_mode(EMIT_MODE_VOLUME),
	m_rate(0.0),
	m_lifetimeRange(std::make_pair(0.0, 0.0)),
	m_velocityRange(std::make_pair(Vector3d::Zero(), Vector3d::Zero())),
	m_particlesNotAdded(0.0),
	m_localPose(SurgSim::Math::RigidTransform3d::Identity()),
	m_logger(SurgSim::Framework::Logger::getLogger("Particles"))
{
	typedef std::pair<double, double> LifetimeRangeType;
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Emitter, LifetimeRangeType, LifetimeRange, getLifetimeRange, setLifetimeRange);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Emitter, int, Mode, getMode, setMode);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Emitter, double, Rate, getRate, setRate);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Emitter, std::shared_ptr<SurgSim::Math::Shape>, Shape, getShape, setShape);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Emitter, std::shared_ptr<SurgSim::Framework::Component>,
									 Target, getTarget, setTarget);
	typedef std::pair<Vector3d, Vector3d> VelocityRangeType;
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Emitter, VelocityRangeType, VelocityRange, getVelocityRange, setVelocityRange);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Emitter, SurgSim::Math::UnalignedRigidTransform3d, LocalPose, getLocalPose,
									  setLocalPose);

	SURGSIM_ADD_RO_PROPERTY(Emitter, SurgSim::Math::UnalignedRigidTransform3d, Pose, getUnalignedPose);

	std::random_device device;
	m_generator.seed(device());
	m_zeroOneDistribution.param(std::uniform_real_distribution<double>::param_type(0.0, 1.0));
}

Emitter::~Emitter()
{
}

bool Emitter::doInitialize()
{
	return true;
}

bool Emitter::doWakeUp()
{
	if (m_target == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Emitters need a Representation to emit to.";
		return false;
	}
	if (m_shape == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Emitters need a shape.";
		return false;
	}
	return true;
}

SurgSim::Math::UnalignedRigidTransform3d Emitter::getUnalignedPose() const
{
	return getPose();
}

void Emitter::update(double dt)
{
	Vector3d position, velocity;
	double lifetime;
	double particlesToAdd = m_rate * dt + m_particlesNotAdded;
	size_t particlesAdded = 0;
	for ( ; particlesAdded < std::floor(particlesToAdd); particlesAdded++)
	{
		if (m_mode == EMIT_MODE_VOLUME)
		{
			position = getPose() * m_pointGenerator.pointInShape(m_shape);
		}
		else
		{
			position = getPose() * m_pointGenerator.pointOnShape(m_shape);
		}

		velocity = Vector3d::NullaryExpr([this](int index){return m_zeroOneDistribution(m_generator);});
		velocity = m_velocityRange.first + (m_velocityRange.second - m_velocityRange.first).cwiseProduct(velocity);

		lifetime = m_zeroOneDistribution(m_generator);
		lifetime = m_lifetimeRange.first + (m_lifetimeRange.second - m_lifetimeRange.first) * lifetime;

		if (!m_target->addParticle(position, velocity, lifetime))
		{
			SURGSIM_LOG_DEBUG(m_logger) << "Unable to add particle to " << m_target->getName();
			break;
		}
	}
	m_particlesNotAdded = particlesToAdd - particlesAdded;
}

int Emitter::getTargetManagerType() const
{
	return SurgSim::Framework::MANAGER_TYPE_PHYSICS;
}

void Emitter::setShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	m_shape = shape;
}

std::shared_ptr<SurgSim::Math::Shape> Emitter::getShape() const
{
	return m_shape;
}

void Emitter::setTarget(const std::shared_ptr<SurgSim::Framework::Component> target)
{
	m_target = SurgSim::Framework::checkAndConvert<SurgSim::Particles::Representation>(target,
			"SurgSim::Particles::Representation");
}

const std::shared_ptr<SurgSim::Framework::Component> Emitter::getTarget()
{
	return m_target;
}

void Emitter::setMode(int mode)
{
	SURGSIM_ASSERT(0 <= mode && mode < EMIT_MODE_COUNT) << "Invalid emit mode";
	m_mode = mode;
}

int Emitter::getMode() const
{
	return m_mode;
}

void Emitter::setRate(double rate)
{
	SURGSIM_ASSERT(rate >= 0.0) << "Emit rate must be non-negative";
	m_rate = rate;
}

double Emitter::getRate() const
{
	return m_rate;
}

void Emitter::setLifetimeRange(const std::pair<double, double>& range)
{
	SURGSIM_ASSERT(0.0 < range.first && range.first <= range.second) <<
		"Lower bound of lifetime must be greater than 0 and not greater than the upper bound of lifetime.";
	m_lifetimeRange = range;
}

std::pair<double, double> Emitter::getLifetimeRange() const
{
	return m_lifetimeRange;
}

void Emitter::setVelocityRange(const std::pair<SurgSim::Math::Vector3d, SurgSim::Math::Vector3d>& range)
{
	SURGSIM_ASSERT((range.first.array() <= range.second.array()).all()) << "Minimum velocity must be less than maximum";
	m_velocityRange = range;
}

const std::pair<SurgSim::Math::Vector3d, SurgSim::Math::Vector3d>& Emitter::getVelocityRange() const
{
	return m_velocityRange;
}

void Emitter::setLocalPose(const SurgSim::Math::UnalignedRigidTransform3d& pose)
{
	m_localPose = pose;
}

SurgSim::Math::RigidTransform3d Emitter::getPose() const
{
	if (getSceneElement() != nullptr)
	{
		return getSceneElement()->getPose() * getLocalPose();
	}
	else
	{
		return getLocalPose();
	}
}

SurgSim::Math::UnalignedRigidTransform3d Emitter::getLocalPose() const
{
	return m_localPose;
}


}; // namespace Particles
}; // namespace SurgSim

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

#include "SurgSim/Particles/EmitterRepresentation.h"

#include <utility>

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Particles/Representation.h"

using SurgSim::Math::Vector3d;


namespace SurgSim
{

namespace Particles
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Particles::EmitterRepresentation, EmitterRepresentation);

EmitterRepresentation::EmitterRepresentation(const std::string& name) :
	SurgSim::Framework::Representation(name),
	m_mode(EMIT_MODE_VOLUME),
	m_rate(0.0),
	m_lifetimeRange(std::make_pair(0.0, 0.0)),
	m_velocityRange(std::make_pair(Vector3d::Zero(), Vector3d::Zero())),
	m_particlesNotAdded(0.0),
	m_logger(SurgSim::Framework::Logger::getLogger("Particles"))
{
	typedef std::pair<double, double> LifetimeRangeType;
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(EmitterRepresentation, LifetimeRangeType, LifetimeRange, getLifetimeRange,
			setLifetimeRange);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(EmitterRepresentation, int, Mode, getMode, setMode);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(EmitterRepresentation, double, Rate, getRate, setRate);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(EmitterRepresentation, std::shared_ptr<SurgSim::Math::Shape>, Shape, getShape,
			setShape);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(EmitterRepresentation, std::shared_ptr<SurgSim::Framework::Component>,
									 Target, getTarget, setTarget);
	typedef std::pair<Vector3d, Vector3d> VelocityRangeType;
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(EmitterRepresentation, VelocityRangeType, VelocityRange, getVelocityRange,
			setVelocityRange);

	std::random_device device;
	m_generator.seed(device());
	m_zeroOneDistribution.param(std::uniform_real_distribution<double>::param_type(0.0, 1.0));
}

EmitterRepresentation::~EmitterRepresentation()
{
}

bool EmitterRepresentation::doInitialize()
{
	return true;
}

bool EmitterRepresentation::doWakeUp()
{
	if (m_target == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "EmitterRepresentations need a Representation to emit to.";
		return false;
	}
	if (m_shape == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "EmitterRepresentations need a shape.";
		return false;
	}
	return true;
}

void EmitterRepresentation::update(double dt)
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

void EmitterRepresentation::setShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	m_shape = shape;
}

std::shared_ptr<SurgSim::Math::Shape> EmitterRepresentation::getShape() const
{
	return m_shape;
}

void EmitterRepresentation::setTarget(const std::shared_ptr<SurgSim::Framework::Component> target)
{
	m_target = SurgSim::Framework::checkAndConvert<SurgSim::Particles::Representation>(target, "SurgSim::Particles::Representation");
}

const std::shared_ptr<SurgSim::Framework::Component> EmitterRepresentation::getTarget()
{
	return m_target;
}

void EmitterRepresentation::setMode(int mode)
{
	SURGSIM_ASSERT(0 <= mode && mode < EMIT_MODE_COUNT) << "Invalid emit mode";
	m_mode = mode;
}

int EmitterRepresentation::getMode() const
{
	return m_mode;
}

void EmitterRepresentation::setRate(double rate)
{
	SURGSIM_ASSERT(rate >= 0.0) << "Emit rate must be non-negative";
	m_rate = rate;
}

double EmitterRepresentation::getRate() const
{
	return m_rate;
}

void EmitterRepresentation::setLifetimeRange(const std::pair<double, double>& range)
{
	SURGSIM_ASSERT(0.0 < range.first && range.first <= range.second) <<
		"Lower bound of lifetime must be greater than 0 and not greater than the upper bound of lifetime.";
	m_lifetimeRange = range;
}

std::pair<double, double> EmitterRepresentation::getLifetimeRange() const
{
	return m_lifetimeRange;
}

void EmitterRepresentation::setVelocityRange(const std::pair<SurgSim::Math::Vector3d, SurgSim::Math::Vector3d>& range)
{
	SURGSIM_ASSERT((range.first.array() <= range.second.array()).all()) << "Minimum velocity must be less than maximum";
	m_velocityRange = range;
}

const std::pair<SurgSim::Math::Vector3d, SurgSim::Math::Vector3d>& EmitterRepresentation::getVelocityRange() const
{
	return m_velocityRange;
}

}; // namespace Particles
}; // namespace SurgSim

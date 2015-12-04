// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Particles/ParticlesCollisionRepresentation.h"

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/ParticlesShape.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Particles/Representation.h"


namespace SurgSim
{
namespace Particles
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Particles::ParticlesCollisionRepresentation,
		ParticlesCollisionRepresentation);

ParticlesCollisionRepresentation::ParticlesCollisionRepresentation(const std::string& name) :
	SurgSim::Collision::Representation(name),
	m_shape(std::make_shared<SurgSim::Math::ParticlesShape>())
{
	m_shape->setRadius(0.01);
}

ParticlesCollisionRepresentation::~ParticlesCollisionRepresentation()
{
}

void ParticlesCollisionRepresentation::update(const double& dt)
{
	*m_shape = getParticleRepresentation()->getParticles().unsafeGet();
	invalidatePosedShape();
}

bool ParticlesCollisionRepresentation::doInitialize()
{
	return true;
}

bool ParticlesCollisionRepresentation::doWakeUp()
{
	auto particleRepresentation = m_particleRepresentation.lock();
	if (particleRepresentation == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << getName()
			<< ": does not have a Particle Representation.";
		return false;
	}

	m_shape->getVertices().reserve(particleRepresentation->getMaxParticles());

	update(0.0);
	return true;
}

int ParticlesCollisionRepresentation::getShapeType() const
{
	return m_shape->getType();
}

const std::shared_ptr<SurgSim::Math::Shape> ParticlesCollisionRepresentation::getShape() const
{
	return m_shape;
}

void ParticlesCollisionRepresentation::setParticleRepresentation(
		std::shared_ptr<SurgSim::Particles::Representation> representation)
{
	m_particleRepresentation = representation;
}

const std::shared_ptr<SurgSim::Particles::Representation> ParticlesCollisionRepresentation::getParticleRepresentation()
		const
{
	auto particleRepresentation = m_particleRepresentation.lock();
	SURGSIM_ASSERT(particleRepresentation != nullptr) <<
		"Failed to get the Particle Representation. The ParticlesCollisionRepresentation either was not "
		"attached to a Particle Representation or the Particle Representation has expired.";

	return particleRepresentation;
}

void ParticlesCollisionRepresentation::setParticleRadius(double radius)
{
	m_shape->setRadius(radius);
}

double ParticlesCollisionRepresentation::getParticleRadius() const
{
	return m_shape->getRadius();
}

};
};


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

#include "SurgSim/Particles/ParticlesCollisionRepresentation.h"

#include "SurgSim/Math/ParticlesShape.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Particles/Particle.h"
#include "SurgSim/Particles/ParticleReference.h"
#include "SurgSim/Particles/ParticleSystemRepresentation.h"

namespace SurgSim
{
namespace Particles
{

ParticlesCollisionRepresentation::ParticlesCollisionRepresentation(const std::string& name) :
	SurgSim::Collision::Representation(name),
	m_shape(std::make_shared<SurgSim::Math::ParticlesShape>())
{
}

ParticlesCollisionRepresentation::~ParticlesCollisionRepresentation()
{
}

void ParticlesCollisionRepresentation::update(const double& dt)
{
	auto particleSystem = getParticleSystem();
	std::list<ParticleReference> particles = particleSystem->getParticleReferences();

	m_shape->getVertices().resize(particles.size());

	auto vertex = m_shape->getVertices().begin();
	auto particle = particles.begin();
	for (; particle != particles.end(); ++particle, ++vertex)
	{
		vertex->position = particle->getPosition();
		vertex->data.index = particle->getIndex();
	}
	m_shape->update();

}

bool ParticlesCollisionRepresentation::doInitialize()
{
	return true;
}

bool ParticlesCollisionRepresentation::doWakeUp()
{
	auto particleSystem = m_particleSystem.lock();
	if (m_particleSystem.lock() == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << getName()
			<< ": does not have a ParticleSystemRepresentation.";
		return false;
	}
	m_shape->getVertices().reserve(particleSystem->getMaxParticles());

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

void ParticlesCollisionRepresentation::setParticleSystem(std::shared_ptr<ParticleSystemRepresentation> representation)
{
	m_particleSystem = representation;
}

const std::shared_ptr<ParticleSystemRepresentation> ParticlesCollisionRepresentation::getParticleSystem() const
{
	auto particleSystem = m_particleSystem.lock();
	SURGSIM_ASSERT(particleSystem != nullptr) <<
		"Failed to get the particle system.  The ParticlesCollisionRepresentation either was not "
		"attached to a ParticleSystemRepresentation or the ParticleSystemRepresentation has expired.";

	return particleSystem;
}


};
};


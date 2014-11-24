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

#include "SurgSim/Particles/ParticleSystemRepresentation.h"

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/Particle.h"
#include "SurgSim/Particles/ParticleReference.h"
#include "SurgSim/Particles/ParticlesState.h"


namespace SurgSim
{
namespace Particles
{

ParticleSystemRepresentation::ParticleSystemRepresentation(const std::string& name) :
	SurgSim::Framework::Representation(name),
	m_maxParticles(0u),
	m_state(std::make_shared<ParticlesState>()),
	m_logger(SurgSim::Framework::Logger::getLogger(name))
{
}

ParticleSystemRepresentation::~ParticleSystemRepresentation()
{
}

bool ParticleSystemRepresentation::doInitialize()
{
	m_state->setNumDof(3, m_maxParticles);
	for (size_t index = 0; index < m_maxParticles; index++)
	{
		m_unusedParticles.emplace_back(m_state, index);
	}
	return true;
}

void ParticleSystemRepresentation::setMaxParticles(size_t maxParticles)
{
	SURGSIM_ASSERT(!isInitialized()) << "Can not set number of particles after initialization";
	m_maxParticles = maxParticles;
}

size_t ParticleSystemRepresentation::getMaxParticles() const
{
	return m_maxParticles;
}

bool ParticleSystemRepresentation::addParticle(const Particle& particle)
{
	SURGSIM_ASSERT(isInitialized()) << "Cannot add particles before initialization";
	bool result;
	std::list<ParticleReference>& particles = m_particles.unsafeGet();
	if (!m_unusedParticles.empty())
	{
		(*m_unusedParticles.begin()) = particle;
		particles.splice(particles.end(), m_unusedParticles, m_unusedParticles.begin());
		result = true;
	}
	else
	{
		SURGSIM_LOG_WARNING(m_logger) << "Unable to add another particle, maximum has been reached ("
			<< m_maxParticles << ").";
		result = false;
	}
	return result;
}

bool ParticleSystemRepresentation::addParticles(const std::vector<Particle>& particles)
{
	bool result = true;
	for (auto particle : particles)
	{
		if (!addParticle(particle))
		{
			result = false;
			break;
		}
	}
	return result;
}

bool ParticleSystemRepresentation::removeParticle(const ParticleReference& particle)
{
	bool result;
	std::list<ParticleReference>& particles = m_particles.unsafeGet();
	auto found = std::find(particles.begin(), particles.end(), particle);
	if (found != particles.end())
	{
		m_unusedParticles.splice(m_unusedParticles.end(), particles, found);
		result = true;
	}
	else
	{
		SURGSIM_LOG_WARNING(m_logger) << "Particle not found, unable to remove";
		result = false;
	}
	return result;
}

ParticleSystemRepresentation::BufferedParticles& ParticleSystemRepresentation::getParticles()
{
	return m_particles;
}

bool ParticleSystemRepresentation::update(double dt)
{
	std::list<ParticleReference>& particles = m_particles.unsafeGet();
	for(auto particleIter = particles.begin(); particleIter != particles.end(); )
	{
		auto nextIter = particleIter;
		nextIter++;
		particleIter->setLifetime(particleIter->getLifetime() - dt);
		if (particleIter->getLifetime() <= 0)
		{
			m_unusedParticles.splice(m_unusedParticles.end(), particles, particleIter);
		}
		particleIter = nextIter;
	}

	bool result = false;
	if (doUpdate(dt))
	{
		m_particles.publish();
		result = true;
	}
	return result;
}

}; // namespace Particles
}; // namespace SurgSim

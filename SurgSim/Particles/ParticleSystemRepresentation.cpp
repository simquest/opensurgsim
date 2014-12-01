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
	m_logger(SurgSim::Framework::Logger::getLogger("Particles"))
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
	if (!m_unusedParticles.empty())
	{
		(*m_unusedParticles.begin()) = particle;
		m_particles.splice(m_particles.end(), m_unusedParticles, m_unusedParticles.begin());
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
	auto found = std::find(m_particles.begin(), m_particles.end(), particle);
	if (found != m_particles.end())
	{
		m_unusedParticles.splice(m_unusedParticles.end(), m_particles, found);
		result = true;
	}
	else
	{
		SURGSIM_LOG_WARNING(m_logger) << "Particle not found, unable to remove";
		result = false;
	}
	return result;
}

std::list<ParticleReference>& ParticleSystemRepresentation::getParticles()
{
	return m_particles;
}

void ParticleSystemRepresentation::update(double dt)
{
	for(auto particleIter = m_particles.begin(); particleIter != m_particles.end(); )
	{
		auto nextIter = particleIter;
		nextIter++;
		particleIter->setLifetime(particleIter->getLifetime() - dt);
		if (particleIter->getLifetime() <= 0)
		{
			m_unusedParticles.splice(m_unusedParticles.end(), m_particles, particleIter);
		}
		particleIter = nextIter;
	}
	SURGSIM_LOG_IF(!doUpdate(dt), m_logger, WARNING) << "Particle System " << getName() << " failed to update.";
}

}; // namespace Particles
}; // namespace SurgSim

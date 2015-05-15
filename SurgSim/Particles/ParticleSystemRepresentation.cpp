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


namespace SurgSim
{
namespace Particles
{

ParticleSystemRepresentation::ParticleSystemRepresentation(const std::string& name) :
	SurgSim::Framework::Representation(name),
	m_maxParticles(0u),
	m_logger(SurgSim::Framework::Logger::getLogger("Particles"))
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(ParticleSystemRepresentation, size_t, MaxParticles, getMaxParticles,
			setMaxParticles);
}

ParticleSystemRepresentation::~ParticleSystemRepresentation()
{
}

bool ParticleSystemRepresentation::doInitialize()
{
	return true;
}

void ParticleSystemRepresentation::setMaxParticles(size_t maxParticles)
{
	m_particles.getVertices().reserve(maxParticles);
	m_maxParticles = maxParticles;
}

size_t ParticleSystemRepresentation::getMaxParticles() const
{
	return m_maxParticles;
}

Particles& ParticleSystemRepresentation::getParticles()
{
	return m_particles;
}

const Particles& ParticleSystemRepresentation::getParticles() const
{
	return m_particles;
}

bool ParticleSystemRepresentation::addParticle(const Particle& particle)
{
	bool result;
	auto& particles = m_particles.getVertices();
	if (particles.size() < m_maxParticles)
	{
		particles.push_back(particle);
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

bool ParticleSystemRepresentation::addParticle(const Math::Vector3d& position, const Math::Vector3d& velocity,
		double lifetime)
{
	ParticleData data = {lifetime, velocity};
	return addParticle(Particle(position, data));
}

void ParticleSystemRepresentation::update(double dt)
{
	auto& particles = m_particles.getVertices();
	auto particle = particles.begin();
	auto newEnd = particles.end();
	while (particle != newEnd)
	{
	   particle->data.lifetime -= dt;
	   if (particle->data.lifetime <= 0.0)
	   {
		   --newEnd;
		   std::swap(*particle, *newEnd);
	   }
	   else
	   {
		   ++particle;
	   }
	}
	particles.erase(newEnd, particles.end());

	if (!doUpdate(dt))
	{
		SURGSIM_LOG_WARNING(m_logger) << "Particle System " << getName() << " failed to update.";
	}
}

}; // namespace Particles
}; // namespace SurgSim

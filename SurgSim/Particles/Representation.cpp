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

#include "SurgSim/Particles/Representation.h"

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/ParticlesCollisionRepresentation.h"


namespace SurgSim
{
namespace Particles
{

Representation::Representation(const std::string& name) :
	SurgSim::Framework::Representation(name),
	m_maxParticles(0u),
	m_logger(SurgSim::Framework::Logger::getLogger("Particles"))
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, size_t, MaxParticles, getMaxParticles,
			setMaxParticles);
}

Representation::~Representation()
{
}

bool Representation::doInitialize()
{
	return true;
}

void Representation::setMaxParticles(size_t maxParticles)
{
	m_particles.getVertices().reserve(maxParticles);
	m_maxParticles = maxParticles;
}

size_t Representation::getMaxParticles() const
{
	return m_maxParticles;
}

Particles& Representation::getParticles()
{
	return m_particles;
}

const Particles& Representation::getParticles() const
{
	return m_particles;
}

bool Representation::addParticle(const Particle& particle)
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

bool Representation::addParticle(const Math::Vector3d& position, const Math::Vector3d& velocity,
		double lifetime)
{
	ParticleData data = {lifetime, velocity};
	return addParticle(Particle(position, data));
}

void Representation::update(double dt)
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

std::shared_ptr<SurgSim::Collision::Representation> Representation::getCollisionRepresentation() const
{
	return m_collisionRepresentation;
}

void Representation::setCollisionRepresentation(std::shared_ptr<SurgSim::Collision::Representation> representation)
{
	if (m_collisionRepresentation != representation)
	{
		auto oldCollisionRep = std::dynamic_pointer_cast<ParticlesCollisionRepresentation>(m_collisionRepresentation);
		if (oldCollisionRep != nullptr)
		{
			oldCollisionRep->setParticleRepresentation(nullptr);
		}
		m_collisionRepresentation = representation;

		auto newCollisionRep = std::dynamic_pointer_cast<ParticlesCollisionRepresentation>(representation);
		if (newCollisionRep != nullptr)
		{
			newCollisionRep->setParticleRepresentation(
					std::static_pointer_cast<SurgSim::Particles::Representation>(getSharedPtr()));
		}
	}
}


}; // namespace Particles
}; // namespace SurgSim

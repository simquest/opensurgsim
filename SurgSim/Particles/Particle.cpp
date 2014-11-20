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

#include "SurgSim/Particles/Particle.h"

#include "SurgSim/Particles/ParticleReference.h"


namespace SurgSim
{
namespace Particles
{

Particle::Particle() :
	m_position(SurgSim::Math::Vector3d::Zero()),
	m_velocity(SurgSim::Math::Vector3d::Zero()),
	m_lifetime(0.0)
{
}

Particle::Particle(const SurgSim::Math::Vector3d& position, const SurgSim::Math::Vector3d& velocity, double lifetime) :
	m_position(position),
	m_velocity(velocity),
	m_lifetime(lifetime)
{
}

Particle::Particle(const ParticleReference& other) :
	m_position(other.getPosition()),
	m_velocity(other.getVelocity()),
	m_lifetime(other.getLifetime())
{
}

void Particle::operator=(const ParticleReference& other)
{
	m_position = other.getPosition();
	m_velocity = other.getVelocity();
	m_lifetime = other.getLifetime();
}

const SurgSim::Math::Vector3d& Particle::getPosition() const
{
	return m_position;
}

void Particle::setPosition(const SurgSim::Math::Vector3d& position)
{
	m_position = position;
}

const SurgSim::Math::Vector3d& Particle::getVelocity() const
{
	return m_velocity;
}

void Particle::setVelocity(const SurgSim::Math::Vector3d& velocity)
{
	m_velocity = velocity;
}

double Particle::getLifetime() const
{
	return m_lifetime;
}

void Particle::setLifetime(double lifetime)
{
	m_lifetime = lifetime;
}

}; // namespace Particles
}; // namespace SurgSim

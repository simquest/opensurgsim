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

#ifndef SURGSIM_PARTICLES_PARTICLE_H
#define SURGSIM_PARTICLES_PARTICLE_H

#include "SurgSim/Math/Vector.h"


namespace SurgSim
{
namespace Particles
{

class ParticleReference;

/// A concrete implementation of a particle
class Particle
{
public:
	/// Constructor
	Particle();

	/// Constructor
	/// \param position The position of the particle
	/// \param velocity The velocity of the particle
	/// \param lifetime The lifetime of the particle
	Particle(const SurgSim::Math::Vector3d& position, const SurgSim::Math::Vector3d& velocity, double lifetime);

	/// Copy Constructor from a ParticleReference
	/// \param other A particle reference to copy from
	explicit Particle(const ParticleReference& other);

	/// Assignment Operator
	/// \param other A particle reference to copy from
	void operator=(const ParticleReference& other);

	/// Get the particle's position
	/// \return The particles position
	const SurgSim::Math::Vector3d& getPosition() const;

	/// Set the particle's position
	/// \param position The particles position
	void setPosition(const SurgSim::Math::Vector3d& position);

	/// Get the particle's velocity
	/// \return The particle's velocity
	const SurgSim::Math::Vector3d& getVelocity() const;

	/// Set the particle's velocity
	/// \param velocity The particle's velocity
	void setVelocity(const SurgSim::Math::Vector3d& velocity);

	/// Get the particle's lifetime
	/// \return The remaining lifetime [s]
	double getLifetime() const;

	/// Set the particle's lifetime
	/// \param lifetime The remaining lifetime [s]
	void setLifetime(double lifetime);

private:
	SurgSim::Math::Vector3d m_position;
	SurgSim::Math::Vector3d m_velocity;
	double m_lifetime;
};

/// Container of Particles
/// This class is a std::vector of Particle with the added ability to copy
/// from containers of compatible type (ie ParticleReference containers).
class Particles : public std::vector<Particle>
{
public:
	/// Copy constructor from another container of compatible type
	template<class R, decltype(typename R::const_iterator(), int()) = 0>
	Particles(const R& other) :
		std::vector<Particle>::vector(other.cbegin(), other.cend())
	{
	}

	/// Constructor
	Particles()
	{
	}
};

}; // namespace Particles
}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_PARTICLE_H

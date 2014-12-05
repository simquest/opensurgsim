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

#ifndef SURGSIM_PARTICLES_PARTICLESYSTEMREPRESENTATION_H
#define SURGSIM_PARTICLES_PARTICLESYSTEMREPRESENTATION_H

#include <boost/thread.hpp>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "SurgSim/Framework/Representation.h"


namespace SurgSim
{

namespace Framework
{
class Logger;
};

namespace Particles
{

class Particle;
class ParticleReference;
class ParticlesState;


/// The ParticleSystemRepresentation class defines the base class for all Particle System.
class ParticleSystemRepresentation : public SurgSim::Framework::Representation
{
public:
	/// Constructor
	/// \param name The representation's name
	explicit ParticleSystemRepresentation(const std::string& name);

	/// Destructor
	virtual ~ParticleSystemRepresentation();

	/// Set the maximum number of particles of this system.
	/// \note Once initialized, it can't be changed.
	/// \param maxParticles The maximum number of particles in this system.
	void setMaxParticles(size_t maxParticles);

	/// \return The number of particles allowed in this system.
	size_t getMaxParticles() const;

	/// Add a particle
	/// Copy the particle into the particle system
	/// \param particle the particle to be added
	/// \return True if the particle was successfully added, false otherwise
	bool addParticle(const Particle& particle);

	/// Add multiple particles
	/// Copy the particles into the particle system
	/// \param particles the particles to be added
	/// \return True if the particles were successfully added, false otherwise
	bool addParticles(const std::vector<Particle>& particles);

	/// Remove particle
	/// \param particle A reference to a particle to remove
	bool removeParticle(const ParticleReference& particle);

	/// Get references to the internal particles
	/// \note this is thread unsafe access to the internal state of the particle system
	/// \return A list of ParticleReference
	std::list<ParticleReference>& getParticleReferences();

	/// Get a copy of the particles in a thread safe manner
	/// \return A vector of particles
	std::shared_ptr<const std::vector<Particle>> getParticles() const;

	/// Update the particle system
	/// \param dt The time step.
	bool update(double dt);

protected:
	/// Maximum amount of particles allowed in this particle system.
	size_t m_maxParticles;

	/// List of particles.
	std::list<ParticleReference> m_particles;

	/// List of unused particles.
	std::list<ParticleReference> m_unusedParticles;

	/// Thread safe copy of the particles
	std::shared_ptr<std::vector<Particle>> m_safeParticles;

	/// The particle system state.
	std::shared_ptr<ParticlesState> m_state;

	/// Logger used by the particle system.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

	/// The mutex used to lock the particles
	mutable boost::shared_mutex m_mutex;

private:
	/// Implementation of the specific behavior of the particle system
	/// \return True if update succeeded, False otherwise.
	virtual bool doUpdate(double dt) = 0;

	virtual bool doInitialize() override;
};

};  // namespace Particles
};  // namespace SurgSim

#endif  // SURGSIM_PARTICLES_PARTICLESYSTEMREPRESENTATION_H

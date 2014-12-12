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

#ifndef SURGSIM_PARTICLES_PARTICLEREFERENCE_H
#define SURGSIM_PARTICLES_PARTICLEREFERENCE_H

#include "SurgSim/Math/Vector.h"


namespace SurgSim
{
namespace Particles
{

class Particle;
class ParticlesState;

/// A reference to a particle contained in a ParticlesState
class ParticleReference
{
public:
	/// Constructor
	/// \param state The ParticlesState this reference refers to
	/// \param index The index of the particle in the ParticlesState
	ParticleReference(std::shared_ptr<ParticlesState> state, size_t index);

	/// Assignment Operator
	/// \param other A Particle to copy from
	void operator=(const Particle& other);

	/// Comparison operator (equality test)
	/// \param other The particle reference to compare to
	/// \return True if the references are equal, False otherwise
	bool operator==(const ParticleReference& other) const;

	/// Get the particle's index
	/// \return The particle's index in the state
	size_t getIndex() const;

	/// Get the particle's position
	/// \return The particles position [m]
	const Eigen::VectorBlock<const SurgSim::Math::Vector, 3> getPosition() const;

	/// Set the particle's position
	/// \param position The particles position [m]
	void setPosition(const Eigen::Ref<const SurgSim::Math::Vector3d>& position);

	/// Get the particle's velocity
	/// \return The particle's velocity [m/s]
	const Eigen::VectorBlock<const SurgSim::Math::Vector, 3> getVelocity() const;

	/// Set the particle's velocity
	/// \param velocity The particle's velocity [m/s]
	void setVelocity(const Eigen::Ref<const SurgSim::Math::Vector3d>& velocity);

	/// Get the particle's lifetime
	/// \return The remaining lifetime [s]
	double getLifetime() const;

	/// Set the particle's lifetime
	/// \param lifetime The remaining lifetime [s]
	void setLifetime(double lifetime);

private:
	const std::shared_ptr<ParticlesState> m_state;
	const size_t m_index;
};

}; // namespace Particles
}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_PARTICLEREFERENCE_H

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

#ifndef SURGSIM_PARTICLES_PARTICLESSTATE_H
#define SURGSIM_PARTICLES_PARTICLESSTATE_H

#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"


namespace SurgSim
{
namespace Particles
{

/// The state of a ParticleSystem
class ParticlesState : public SurgSim::Math::OdeState
{
public:
	/// Constructor
	ParticlesState();

	/// Comparison operator (equality test)
	/// \param other The state to compare it to
	/// \return True if the 2 states are equal, False otherwise
	bool operator==(const ParticlesState& other) const;

	/// Comparison operator (difference test)
	/// \param other The state to compare it to
	/// \return False if the 2 states are equal, True otherwise
	bool operator!=(const ParticlesState& other) const;

	/// Retrieves all lifetimes (non-const version)
	/// \return Vector of collected lifetimes
	SurgSim::Math::Vector& getLifetimes();

	/// Retrieves all lifetimes (const version)
	/// \return Vector of collected lifetimes
	const SurgSim::Math::Vector& getLifetimes() const;

	/// Retrieves all accelerations (non-const version)
	/// \return Vector of collected accelerations
	SurgSim::Math::Vector& getAccelerations();

	/// Retrieves all accelerations (const version)
	/// \return Vector of collected accelerations
	const SurgSim::Math::Vector& getAccelerations() const;

	virtual void reset() override;

	virtual void setNumDof(size_t numDofPerNode, size_t numNodes) override;

	virtual bool isValid() const override;

private:
	SurgSim::Math::Vector m_lifetimes;

	SurgSim::Math::Vector m_accelerations;
};


}; // namespace Particles
}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_PARTICLESSTATE_H

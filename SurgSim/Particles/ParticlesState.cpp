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

#include "SurgSim/Particles/ParticlesState.h"

#include "SurgSim/Math/Valid.h"


namespace SurgSim
{

namespace Particles
{

ParticlesState::ParticlesState()
{
}

bool ParticlesState::operator ==(const ParticlesState& other) const
{
	return OdeState::operator==(other) && m_lifetimes == other.m_lifetimes;
}

bool ParticlesState::operator !=(const ParticlesState& other) const
{
	return !((*this) == other);
}

SurgSim::Math::Vector& ParticlesState::getLifetimes()
{
	return m_lifetimes;
}

const SurgSim::Math::Vector& ParticlesState::getLifetimes() const
{
	return m_lifetimes;
}

void ParticlesState::reset()
{
	OdeState::reset();
	m_lifetimes.setZero();
}

void ParticlesState::setNumDof(size_t numDofPerNode, size_t numNodes)
{
	m_lifetimes.resize(numNodes);
	OdeState::setNumDof(numDofPerNode, numNodes);
}

bool ParticlesState::isValid() const
{
	return OdeState::isValid() && SurgSim::Math::isValid(getLifetimes());
}

}; // namespace Particles
}; // namespace SurgSim


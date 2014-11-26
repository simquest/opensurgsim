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

#include "SurgSim/Particles/UnitTests/MockObjects.h"


namespace SurgSim
{
namespace Particles
{

MockParticleSystem::MockParticleSystem(const std::string& name) :
	ParticleSystemRepresentation(name),
	updateCount(0)
{
}

bool MockParticleSystem::doUpdate(double dt)
{
	updateCount++;
	return true;
}

MockEmitter::MockEmitter(const std::string& name) :
	EmitterRepresentation(name),
	updateCount(0)
{
}

void MockEmitter::update(double dt)
{
	EmitterRepresentation::update(dt);
	updateCount++;
}

MockParticleBehavior::MockParticleBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	updateCount(0)
{
}

bool MockParticleBehavior::doInitialize()
{
	return true;
}

bool MockParticleBehavior::doWakeUp()
{
	return true;
}

void MockParticleBehavior::update(double dt)
{
	updateCount++;
}

int MockParticleBehavior::getTargetManagerType() const
{
	return SurgSim::Framework::MANAGER_TYPE_PARTICLES;
}

}; // namespace Particles
}; // namespace SurgSim

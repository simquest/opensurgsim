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

#include "SurgSim/Particles/ParticleManager.h"

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Particles/EmitterRepresentation.h"
#include "SurgSim/Particles/Representation.h"


namespace SurgSim
{
namespace Particles
{

ParticleManager::ParticleManager() : ComponentManager("Particle Manager")
{
	m_logger = SurgSim::Framework::Logger::getLogger("Particles");
}

ParticleManager::~ParticleManager()
{
}

bool ParticleManager::doInitialize()
{
	return true;
}

bool ParticleManager::doStartUp()
{
	return true;
}

bool ParticleManager::executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	std::shared_ptr<Representation> particleSystem = tryAddComponent(component, &m_particleSystems);
	std::shared_ptr<EmitterRepresentation> emitter = tryAddComponent(component, &m_emitters);
	return particleSystem != nullptr || emitter!= nullptr;
}

bool ParticleManager::executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	return tryRemoveComponent(component, &m_particleSystems) || tryRemoveComponent(component, &m_emitters);
}

bool ParticleManager::doUpdate(double dt)
{
	processComponents();
	processBehaviors(dt);

	for (auto emitter : m_emitters)
	{
		if (emitter->isActive())
		{
			emitter->update(dt);
		}
	}

	for (auto particleSystem : m_particleSystems)
	{
		if (particleSystem->isActive())
		{
			particleSystem->update(dt);
		}
	}

	return true;
}

int ParticleManager::getType() const
{
	return SurgSim::Framework::MANAGER_TYPE_PARTICLES;
}

}; // namespace Particles
}; // namespace SurgSim

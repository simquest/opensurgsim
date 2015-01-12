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

#ifndef SURGSIM_PARTICLES_PARTICLEMANAGER_H
#define SURGSIM_PARTICLES_PARTICLEMANAGER_H

#include <memory>

#include "SurgSim/Framework/ComponentManager.h"


namespace SurgSim
{

namespace Framework
{
class Component;
};

namespace Particles
{

class EmitterRepresentation;
class ParticleSystemRepresentation;

class ParticleManager: public SurgSim::Framework::ComponentManager
{
public:
	/// Constructor
	ParticleManager();

	/// Destructor
	virtual ~ParticleManager();

	int getType() const override;

protected:
	bool executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component) override;
	bool executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component) override;

private:
	bool doInitialize() override;
	bool doStartUp() override;
	bool doUpdate(double dt) override;

	/// ParticleSystemRepresentations managed by this manager
	std::vector<std::shared_ptr<ParticleSystemRepresentation>> m_particleSystems;

	/// EmitterRepresentations managed by this manager
	std::vector<std::shared_ptr<EmitterRepresentation>> m_emitters;
};


}; // namespace Particles
}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_PARTICLEMANAGER_H

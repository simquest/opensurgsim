// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PARTICLES_SINK_H
#define SURGSIM_PARTICLES_SINK_H

#include <memory>

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/ObjectFactory.h"


namespace SurgSim
{

namespace Collision
{
class Representation;
};

namespace Framework
{
class Component;
class Logger;
};

namespace Particles
{

class Representation;

SURGSIM_STATIC_REGISTRATION(Sink);

/// Sink removes particles from a ParticleSystem
class Sink : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param name The Sink's name
	explicit Sink(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Particles::Sink);

	/// Set the target to remove particles from
	/// \param target The ParticleSystem to remove from.
	void setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target);

	/// Get the target removing particles from
	/// \return The ParticleSystem to remove from.
	std::shared_ptr<SurgSim::Framework::Component> getTarget();

	/// Set the collision representation for this Sink
	/// Particles that collide with this collision representation will be removed
	/// \param representation The collision representation
	void setCollisionRepresentation(const std::shared_ptr<SurgSim::Framework::Component>& representation);

	/// Get the collision representation for this Sink
	/// Particles that collide with this collision representation will be removed
	/// \return the collision representation for this Sink
	std::shared_ptr<SurgSim::Framework::Component> getCollisionRepresentation();

	void update(double dt) override;

	int getTargetManagerType() const override;

private:
	bool doInitialize() override;

	bool doWakeUp() override;

	std::shared_ptr<SurgSim::Collision::Representation> m_collisionRepresentation;

	std::shared_ptr<SurgSim::Particles::Representation> m_target;

	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
};

};
};

#endif // SURGSIM_PARTICLES_SINK_H


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


#include "SurgSim/Particles/Sink.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Particles/Representation.h"


namespace SurgSim
{

namespace Particles
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Particles::Sink, Sink);

Sink::Sink(const std::string& name) :
	Framework::Behavior(name),
	m_logger(Framework::Logger::getLogger("Particles"))
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Sink, std::shared_ptr<SurgSim::Framework::Component>, Target, getTarget,
			setTarget);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Sink, std::shared_ptr<SurgSim::Framework::Component>, CollisionRepresentation,
			getCollisionRepresentation, setCollisionRepresentation);
}

void Sink::setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target)
{
	m_target = Framework::checkAndConvert<SurgSim::Particles::Representation>(target,
			"SurgSim::Particles::Representation");
}

std::shared_ptr<SurgSim::Framework::Component> Sink::getTarget()
{
	return m_target;
}

void Sink::setCollisionRepresentation(const std::shared_ptr<Framework::Component>& representation)
{
	m_collisionRepresentation = Framework::checkAndConvert<SurgSim::Collision::Representation>(representation,
			"SurgSim::Collision::Representation");
}

std::shared_ptr<SurgSim::Framework::Component> Sink::getCollisionRepresentation()
{
	return m_collisionRepresentation;
}

bool Sink::doInitialize()
{
	return true;
}

bool Sink::doWakeUp()
{
	if (m_collisionRepresentation == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Sinks need a Collision Representation.";
		return false;
	}
	if (m_target == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Sinks need a Particle Representation to remove from.";
		return false;
	}
	return true;
}

void Sink::update(double dt)
{
	auto collisions = m_collisionRepresentation->getCollisions().safeGet();
	auto found = collisions->find(m_target->getCollisionRepresentation());
	if (found != collisions->end())
	{
		for (auto& contact : found->second)
		{
			m_target->removeParticle(contact->penetrationPoints.second.index.getValue());
		}
	}
}

int Sink::getTargetManagerType() const
{
	return Framework::MANAGER_TYPE_PHYSICS;
}


};
};

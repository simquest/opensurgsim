#include "Actor.h"

#include "ActorImplementation.h"

using SurgSim::Graphics::Actor;
using SurgSim::Graphics::ActorImplementation;

Actor::Actor(const std::string& name, std::shared_ptr<ActorImplementation> implementation) : m_name(name), m_implementation(implementation)
{
}

Actor::~Actor()
{
}

void Actor::doUpdate(double dt)
{
	m_implementation->update(dt);
}
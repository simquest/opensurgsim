#include "Actor.h"

#include "ActorImplementation.h"

using SurgSim::Framework::Representation;
using SurgSim::Physics::Actor;
using SurgSim::Physics::ActorImplementation;

Actor::Actor(const std::string& name, std::shared_ptr<ActorImplementation> implementation) : Representation(name), m_implementation(implementation)
{
}

Actor::~Actor()
{
}
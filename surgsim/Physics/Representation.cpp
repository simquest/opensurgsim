#include "Representation.h"

#include "Actor.h"

using SurgSim::Physics::Actor;
using SurgSim::Physics::Representation;

Representation::Representation(std::shared_ptr<Actor> actor) : Framework::Representation(actor->getName()), 
	m_actor(actor)
{
}
Representation::~Representation()
{
}
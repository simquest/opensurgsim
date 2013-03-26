#include "RigidActor.h"

#include <SurgSim/Graphics/RigidActorImplementation.h>

using SurgSim::Graphics::Actor;
using SurgSim::Graphics::RigidActor;
using SurgSim::Graphics::RigidActorImplementation;

RigidActor::RigidActor(const std::string& name, std::shared_ptr<RigidActorImplementation> implementation) : 
Actor(name, implementation)
{
}

RigidActor::~RigidActor()
{
}

void RigidActor::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	getRigidImplementation()->setPose(pose);
}

const SurgSim::Math::RigidTransform3d& RigidActor::getPose() const
{
	return getRigidImplementation()->getPose();
}

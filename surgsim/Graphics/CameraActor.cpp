#include "CameraActor.h"

#include <SurgSim/Graphics/CameraImplementation.h>

using SurgSim::Graphics::Actor;
using SurgSim::Graphics::CameraActor;
using SurgSim::Graphics::CameraImplementation;

CameraActor::CameraActor(const std::string& name, std::shared_ptr<CameraImplementation> implementation) : 
	Actor(name, implementation)
{
}

CameraActor::~CameraActor()
{
}

std::shared_ptr<CameraImplementation> CameraActor::getCameraImplementation() const
{
	return std::static_pointer_cast<CameraImplementation>(getImplementation());
}
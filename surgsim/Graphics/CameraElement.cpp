#include "CameraElement.h"

#include <SurgSim/Graphics/CameraActor.h>

using SurgSim::Graphics::CameraActor;
using SurgSim::Graphics::CameraElement;
using SurgSim::Graphics::Representation;


CameraElement::CameraElement(std::shared_ptr<CameraActor> actor) : 
	Representation(actor)
{
}

CameraElement::~CameraElement()
{
}

std::shared_ptr<CameraActor> CameraElement::getCameraActor() const
{
	return std::static_pointer_cast<CameraActor>(getActor());
}
#include "ViewElement.h"

#include <SurgSim/Graphics/CameraElement.h>
#include <SurgSim/Graphics/ViewComponent.h>

using SurgSim::Graphics::CameraElement;
using SurgSim::Graphics::ViewElement;
using SurgSim::Graphics::ViewComponent;

ViewElement::ViewElement(const std::string& name, std::shared_ptr<SurgSim::Graphics::ViewComponent> component) : 
	SceneElement(name),
	m_view(component)
{
	addComponent(component);
	m_camera = std::make_shared<CameraElement>(m_view->getCamera());
}
ViewElement::~ViewElement()
{

}

bool ViewElement::setCamera(std::shared_ptr<CameraElement> camera)
{
	return m_view->setCamera(camera->getCameraActor());
}

void ViewElement::setScreen(unsigned int screen)
{
	m_view->setScreen(screen);
}
void ViewElement::setFullScreen(bool fullscreen)
{
	m_view->setFullScreen(fullscreen);
}
void ViewElement::setWindow(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	m_view->setWindow(x, y, width, height);
}

bool ViewElement::doInitialize()
{
	return true;
}
bool ViewElement::doWakeUp()
{
	return true;
}
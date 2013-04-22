#include "ViewComponent.h"

#include <SurgSim/Graphics/CameraActor.h>
#include <SurgSim/Graphics/ViewImplementation.h>

using SurgSim::Graphics::CameraActor;
using SurgSim::Graphics::ViewComponent;
using SurgSim::Graphics::ViewImplementation;

ViewComponent::ViewComponent(const std::string& name, std::shared_ptr<ViewImplementation> implementation) : 
Component(name), 
	m_implementation(implementation),
	m_screen(0), 
	m_isFullscreen(false),
	m_windowX(100),
	m_windowY(100),
	m_windowWidth(800),
	m_windowHeight(600),
	m_isFirstUpdate(true)
{
	m_camera = std::make_shared<CameraActor>(name + " Camera", m_implementation->getCamera());
}
ViewComponent::~ViewComponent()
{
}

bool ViewComponent::setCamera(std::shared_ptr<CameraActor> camera)
{
	m_camera = camera;
	return m_implementation->setCamera(camera->getCameraImplementation());
}

void ViewComponent::setScreen(unsigned int screen)
{
	m_screen = screen;
	updateWindow();
}
void ViewComponent::setFullScreen(bool fullscreen)
{
	m_isFullscreen = fullscreen;
	updateWindow();
}
void ViewComponent::setWindow(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	m_windowX = x;
	m_windowY = y;
	m_windowWidth = width;
	m_windowHeight = height;
	updateWindow();
}

bool ViewComponent::doInitialize()
{
	return true;
}
bool ViewComponent::doWakeUp()
{
	return true;
}

void ViewComponent::doBeforeFirstUpdate(double dt)
{
	if (m_isFullscreen)
	{
		m_implementation->setUpFullScreen(m_screen);
	}
	else
	{
		m_implementation->setUpWindow(m_windowX, m_windowY, m_windowWidth, m_windowHeight, m_screen);
	}
}
void ViewComponent::doUpdate(double dt)
{
	m_implementation->update(dt);
}

void ViewComponent::updateWindow()
{
	if (m_isFullscreen)
	{
		m_implementation->makeFullScreened(m_screen);
	}
	else
	{
		m_implementation->makeWindowed(m_windowX, m_windowY, m_windowWidth, m_windowHeight, m_screen);
	}
}
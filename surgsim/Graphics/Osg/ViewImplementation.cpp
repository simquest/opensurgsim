#include "ViewImplementation.h"

#include <SurgSim/Graphics/Osg/CameraImplementation.h>

using SurgSim::Graphics::Osg::CameraImplementation;
using SurgSim::Graphics::Osg::ViewImplementation;

ViewImplementation::ViewImplementation() : SurgSim::Graphics::ViewImplementation(), m_viewer(new osgViewer::Viewer)
{
	std::shared_ptr<CameraImplementation> camera = std::make_shared<CameraImplementation>();
	setCamera(camera);
}
ViewImplementation::~ViewImplementation()
{
}

bool ViewImplementation::setCamera(std::shared_ptr<SurgSim::Graphics::CameraImplementation> camera)
{
	bool result = false;
	std::shared_ptr<CameraImplementation> osgCamera = 
		std::dynamic_pointer_cast<CameraImplementation>(camera);
	if (osgCamera != nullptr)
	{
		m_viewer->setCamera(osgCamera->getCamera());
		result = true;
	}
	return result;
}

bool ViewImplementation::setUpFullScreen(unsigned int screen)
{
	m_viewer->setUpViewOnSingleScreen(screen);
	m_viewer->realize();
	return true;
}
bool ViewImplementation::setUpWindow(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned int screen)
{
	m_viewer->setUpViewInWindow(x, y, width, height, screen);
	m_viewer->realize();
	return true;
}

bool ViewImplementation::makeFullScreened(unsigned int screen)
{
	osgViewer::GraphicsWindow* window = static_cast<osgViewer::GraphicsWindow*>(m_viewer->getCamera()->getGraphicsContext());
	osg::GraphicsContext::WindowingSystemInterface* windowingSystem = osg::GraphicsContext::getWindowingSystemInterface();
	unsigned int screenWidth;
	unsigned int screenHeight;
	windowingSystem->getScreenResolution(*(window->getTraits()), screenWidth, screenHeight);

	window->setWindowDecoration(false);
	window->setWindowRectangle(0, 0, screenWidth, screenHeight);
	return true;
}
bool ViewImplementation::makeWindowed(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned int screen)
{
	osgViewer::GraphicsWindow* window = static_cast<osgViewer::GraphicsWindow*>(m_viewer->getCamera()->getGraphicsContext());

	window->setWindowDecoration(true);
	window->setWindowRectangle(x, y, width, height);
	return true;
}

void ViewImplementation::doUpdate(double dt)
{
	m_viewer->frame();
}
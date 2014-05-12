// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include "SurgSim/Graphics/OsgView.h"


#include "SurgSim/Devices/Keyboard/KeyboardDevice.h"
#include "SurgSim/Devices/Keyboard/OsgKeyboardHandler.h"
#include "SurgSim/Devices/Mouse/MouseDevice.h"
#include "SurgSim/Devices/Mouse/OsgMouseHandler.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Graphics/OsgTrackballZoomManipulator.h"

#include <osgViewer/ViewerEventHandlers>

namespace SurgSim
{
namespace Graphics
{

OsgView::OsgView(const std::string& name) : View(name),
	m_x(0), m_y(0),
	m_width(800), m_height(600),
	m_isWindowBorderEnabled(true),
	m_isFirstUpdate(true),
	m_areWindowSettingsDirty(false),
	m_view(new osgViewer::View())
{
	/// Don't allow the default camera here, let that be handled at a higher level.
	m_view->setCamera(nullptr);
}

bool OsgView::setPosition(int x, int y)
{
	if (x != m_x || y != m_y)
	{
		m_areWindowSettingsDirty = true;
	}
	m_x = x;
	m_y = y;
	return true;
}

void OsgView::getPosition(int* x, int* y) const
{
	*x = m_x;
	*y = m_y;
}

bool OsgView::setDimensions(int width, int height)
{
	if (width != m_width || height != m_height)
	{
		m_areWindowSettingsDirty = true;
	}
	m_width = width;
	m_height = height;
	return true;
}

void OsgView::getDimensions(int* width, int* height) const
{
	*width = m_width;
	*height = m_height;
}

void OsgView::setWindowBorderEnabled(bool enabled)
{
	m_isWindowBorderEnabled = enabled;
	m_areWindowSettingsDirty = true;
}

bool OsgView::isWindowBorderEnabled() const
{
	return m_isWindowBorderEnabled;
}

bool OsgView::setCamera(std::shared_ptr<SurgSim::Graphics::Camera> camera)
{
	std::shared_ptr<OsgCamera> osgCamera = std::dynamic_pointer_cast<OsgCamera>(camera);
	if (osgCamera != nullptr && View::setCamera(camera))
	{
		m_view->setCamera(osgCamera->getOsgCamera());
		return true;
	}
	else
	{
		return false;
	}
}

void OsgView::update(double dt)
{
	if (m_areWindowSettingsDirty)
	{
		osg::Camera* viewCamera = m_view->getCamera();
		if (viewCamera)
		{
			osgViewer::GraphicsWindow* window =
				dynamic_cast<osgViewer::GraphicsWindow*>(viewCamera->getGraphicsContext());
			if (window)
			{
				window->setWindowDecoration(m_isWindowBorderEnabled);
				window->setWindowRectangle(m_x, m_y, m_width, m_height);
				m_areWindowSettingsDirty = false;
			}
		}
	}
}

bool OsgView::doInitialize()
{
	return true;
}

bool OsgView::doWakeUp()
{
	m_view->setUpViewInWindow(m_x, m_y, m_width, m_height);
	m_view->addEventHandler(new osgViewer::StatsHandler);
	return true;
}

OsgView::~OsgView()
{
	// Clean up the handler callbacks
	enableKeyboardDevice(false);
	enableMouseDevice(false);
}

osg::ref_ptr<osgViewer::View> SurgSim::Graphics::OsgView::getOsgView() const
{
	return m_view;
}

void SurgSim::Graphics::OsgView::enableManipulator(bool val)
{
	if (m_manipulator == nullptr)
	{
		m_manipulator = new SurgSim::Graphics::OsgTrackballZoomManipulator();
		// Set a default position
		m_manipulator->setTransformation(
			SurgSim::Graphics::toOsg(m_manipulatorPosition),
			SurgSim::Graphics::toOsg(m_manipulatorLookat),
			osg::Vec3d(0.0f, 1.0f, 0.0f));
	}

	if (val)
	{
		getOsgView()->setCameraManipulator(m_manipulator);
	}
	else
	{
		getOsgView()->setCameraManipulator(nullptr);
	}
}

void SurgSim::Graphics::OsgView::enableKeyboardDevice(bool val)
{
	// Early return if device is already turned on/off.
	if (val == m_keyboardEnabled)
	{
		return;
	}

	std::shared_ptr<SurgSim::Input::CommonDevice> keyboardDevice = getKeyboardDevice();
	osg::ref_ptr<osgGA::GUIEventHandler> keyboardHandle =
		std::static_pointer_cast<SurgSim::Device::KeyboardDevice>(keyboardDevice)->getKeyboardHandler();
	if (val)
	{
		getOsgView()->addEventHandler(keyboardHandle);
		m_keyboardEnabled = true;
	}
	else
	{
		getOsgView()->removeEventHandler(keyboardHandle);
		m_keyboardEnabled = false;
	}
}


std::shared_ptr<SurgSim::Input::CommonDevice> SurgSim::Graphics::OsgView::getKeyboardDevice()
{
	static auto keyboardDevice = std::make_shared<SurgSim::Device::KeyboardDevice>("Keyboard");
	if (!keyboardDevice->isInitialized())
	{
		keyboardDevice->initialize();
	}
	return keyboardDevice;
}

void SurgSim::Graphics::OsgView::enableMouseDevice(bool val)
{
	// Early return if device is already turned on/off.
	if (val == m_mouseEnabled)
	{
		return;
	}

	std::shared_ptr<SurgSim::Input::CommonDevice> mouseDevice = getMouseDevice();
	osg::ref_ptr<osgGA::GUIEventHandler> mouseHandler =
		std::static_pointer_cast<SurgSim::Device::MouseDevice>(mouseDevice)->getMouseHandler();
	if (val)
	{
		getOsgView()->addEventHandler(mouseHandler);
		m_mouseEnabled = true;
	}
	else
	{
		getOsgView()->removeEventHandler(mouseHandler);
		m_mouseEnabled = false;
	}
}


std::shared_ptr<SurgSim::Input::CommonDevice> SurgSim::Graphics::OsgView::getMouseDevice()
{
	static auto mouseDevice = std::make_shared<SurgSim::Device::MouseDevice>("Mouse");
	if (!mouseDevice->isInitialized())
	{
		mouseDevice->initialize();
	}
	return mouseDevice;
}


void SurgSim::Graphics::OsgView::setManipulatorParameters(
	SurgSim::Math::Vector3d position,
	SurgSim::Math::Vector3d lookat)
{
	m_manipulatorPosition = position;
	m_manipulatorLookat = lookat;

	if (m_manipulator != nullptr)
	{
		m_manipulator->setTransformation(
			SurgSim::Graphics::toOsg(m_manipulatorPosition),
			SurgSim::Graphics::toOsg(m_manipulatorLookat),
			osg::Vec3d(0.0f, 1.0f, 0.0f));
	}
}

}
}
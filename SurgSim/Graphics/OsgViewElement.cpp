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

#include "SurgSim/Devices/Keyboard/KeyboardDevice.h"
#include "SurgSim/Devices/Keyboard/KeyboardHandler.h"

#include "SurgSim/Devices/Mouse/MouseDevice.h"
#include "SurgSim/Devices/Mouse/MouseHandler.h"

#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgTrackballZoomManipulator.h"


using SurgSim::Graphics::OsgView;
using SurgSim::Graphics::OsgViewElement;

OsgViewElement::OsgViewElement(const std::string& name) :
	SurgSim::Graphics::ViewElement(name, std::make_shared<OsgView>(name + " View")),
	m_manipulatorPosition(SurgSim::Math::Vector3d(3.0, 3.0, 3.0)),
	m_manipulatorLookat(SurgSim::Math::Vector3d(0.0, 0.0, 0.0)),
	m_keyboardEnabled(false),
	m_mouseEnabled(false)
{
}

OsgViewElement::~OsgViewElement()
{
}

bool OsgViewElement::setView(std::shared_ptr<SurgSim::Graphics::View> view)
{
	bool result = false;
	if (getView() == view)
	{
		result = true;
	}
	else
	{
		// Disable KeyboardDevice of current view
		result = enableKeyboardDevice(false);
		result = enableMouseDevice(false) && result;

		std::shared_ptr<OsgView> osgView = std::dynamic_pointer_cast<OsgView>(view);
		if (osgView && ViewElement::setView(view) && result)
		{
			// After change 'view', need to enable keyboard/mouse in the new 'view' if they were enabled.
			result = enableKeyboardDevice(m_keyboardEnabled);
			result = enableMouseDevice(m_mouseEnabled) && result;
		}
		else
		{
			result = false;
		}
	}

	return result;
}

void SurgSim::Graphics::OsgViewElement::enableManipulator(bool val)
{
	if (m_manipulator == nullptr)
	{
		m_manipulator = new OsgTrackballZoomManipulator();
		// Set a default position
		m_manipulator->setTransformation(
			SurgSim::Graphics::toOsg(m_manipulatorPosition),
			SurgSim::Graphics::toOsg(m_manipulatorLookat),
			osg::Vec3d(0.0f,1.0f,0.0f));
	}

	std::shared_ptr<OsgView> view = std::dynamic_pointer_cast<OsgView>(getView());
	if (view != nullptr)
	{
		if (val)
		{
			view->getOsgView()->setCameraManipulator(m_manipulator);

		}
		else
		{
			view->getOsgView()->setCameraManipulator(nullptr);
		}
	}
}

bool SurgSim::Graphics::OsgViewElement::enableKeyboardDevice(bool val)
{
	bool result = false;
	if (val == m_keyboardEnabled)
	{
		result = true;
	}
	else // val != m_keyboardEnabled
	{
		std::shared_ptr<OsgView> view = std::dynamic_pointer_cast<OsgView>(getView());
		if (nullptr != view)
		{
			std::shared_ptr<SurgSim::Input::CommonDevice> keyboardDevice = getKeyboardDevice();
			osg::ref_ptr<osgGA::GUIEventHandler> keyboardHandle =
					std::dynamic_pointer_cast<SurgSim::Device::KeyboardDevice>(keyboardDevice)->getKeyboardHandler();
			if (val) // Turn on keyboard
			{
				view->getOsgView()->addEventHandler(keyboardHandle);
				m_keyboardEnabled = true;
			}
			else // Turn off keyboard
			{
				view->getOsgView()->removeEventHandler(keyboardHandle);
				m_keyboardEnabled = false;
			}
			result = true;
		}
	}

	return result;
}


std::shared_ptr<SurgSim::Input::CommonDevice> SurgSim::Graphics::OsgViewElement::getKeyboardDevice()
{
	static auto keyboardDevice = std::make_shared<SurgSim::Device::KeyboardDevice>("Keyboard");
	if (!keyboardDevice->isInitialized())
	{
		keyboardDevice->initialize();
	}
	return keyboardDevice;
}

bool SurgSim::Graphics::OsgViewElement::enableMouseDevice(bool val)
{
	bool result = false;
	if (val == m_mouseEnabled)
	{
		result = true;
	}
	else // val != m_mouseEnabled
	{
		std::shared_ptr<OsgView> view = std::dynamic_pointer_cast<OsgView>(getView());
		if (nullptr != view)
		{
			std::shared_ptr<SurgSim::Input::CommonDevice> mouseDevice = getMouseDevice();
			osg::ref_ptr<osgGA::GUIEventHandler> mouseHandler =
				std::dynamic_pointer_cast<SurgSim::Device::MouseDevice>(mouseDevice)->getMouseHandler();
			if (val) // Turn on mouse device
			{
				view->getOsgView()->addEventHandler(mouseHandler);
				m_mouseEnabled = true;
			}
			else // Turn off mouse device
			{
				view->getOsgView()->removeEventHandler(mouseHandler);
				m_mouseEnabled = false;
			}
			result = true;
		}
	}

	return result;
}


std::shared_ptr<SurgSim::Input::CommonDevice> SurgSim::Graphics::OsgViewElement::getMouseDevice()
{
	static auto mouseDevice = std::make_shared<SurgSim::Device::MouseDevice>("Mouse");
	if (!mouseDevice->isInitialized())
	{
		mouseDevice->initialize();
	}
	return mouseDevice;
}


void SurgSim::Graphics::OsgViewElement::setManipulatorParameters(
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
			osg::Vec3d(0.0f,1.0f,0.0f));
	}
}

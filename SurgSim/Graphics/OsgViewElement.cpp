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

#include "SurgSim/Graphics/OsgViewElement.h"

#include "SurgSim/Devices/Keyboard/KeyboardDevice.h"
#include "SurgSim/Devices/Keyboard/OsgKeyboardHandler.h"
#include "SurgSim/Devices/Mouse/MouseDevice.h"
#include "SurgSim/Devices/Mouse/OsgMouseHandler.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Graphics/OsgTrackballZoomManipulator.h"
#include "SurgSim/Graphics/OsgView.h"

using SurgSim::Graphics::OsgCamera;
using SurgSim::Graphics::OsgView;
using SurgSim::Graphics::OsgViewElement;

OsgViewElement::OsgViewElement(const std::string& name) :
	SurgSim::Graphics::ViewElement(name),
	m_keyboardEnabled(false),
	m_mouseEnabled(false)
{
	setView(std::make_shared<OsgView>("View"));
	setCamera(std::make_shared<OsgCamera>("Camera"));

	getCamera()->setRenderGroupReference(Representation::DefaultGroupName);
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
		std::shared_ptr<OsgView> osgView = std::dynamic_pointer_cast<OsgView>(view);
		if (osgView)
		{
			result = ViewElement::setView(view);
		}
	}

	return result;
}

void SurgSim::Graphics::OsgViewElement::enableManipulator(bool val)
{

	std::shared_ptr<OsgView> osgView = std::static_pointer_cast<OsgView>(getView());
	osgView->enableManipulator(val);

}

void SurgSim::Graphics::OsgViewElement::enableKeyboardDevice(bool val)
{
	std::shared_ptr<OsgView> osgView = std::static_pointer_cast<OsgView>(getView());
	osgView->enableKeyboardDevice(val);
}


std::shared_ptr<SurgSim::Input::CommonDevice> SurgSim::Graphics::OsgViewElement::getKeyboardDevice()
{
	std::shared_ptr<OsgView> osgView = std::static_pointer_cast<OsgView>(getView());
	return osgView->getKeyboardDevice();
}

void SurgSim::Graphics::OsgViewElement::enableMouseDevice(bool val)
{
	std::shared_ptr<OsgView> osgView = std::static_pointer_cast<OsgView>(getView());
	osgView->enableMouseDevice(val);
}


std::shared_ptr<SurgSim::Input::CommonDevice> SurgSim::Graphics::OsgViewElement::getMouseDevice()
{
	std::shared_ptr<OsgView> osgView = std::static_pointer_cast<OsgView>(getView());
	return osgView->getMouseDevice();
}


void SurgSim::Graphics::OsgViewElement::setManipulatorParameters(const SurgSim::Math::Vector3d& position,
		const SurgSim::Math::Vector3d& lookat)
{
	std::shared_ptr<OsgView> osgView = std::static_pointer_cast<OsgView>(getView());
	osgView->setManipulatorParameters(position, lookat);
}

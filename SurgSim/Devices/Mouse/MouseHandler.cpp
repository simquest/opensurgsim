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

#include "SurgSim/Devices/Mouse/MouseHandler.h"
#include "SurgSim/Devices/Mouse/MouseScaffold.h"

#include <memory>

#include <osgGA/GUIEventHandler>

namespace SurgSim
{
namespace Device
{

MouseHandler::MouseHandler() : m_mouseScaffold(MouseScaffold::getOrCreateSharedInstance())
{
}

bool MouseHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
	bool handled = false;

	auto eventType = ea.getEventType();
	switch(eventType)
	{
	case(osgGA::GUIEventAdapter::SCROLL) :
	{
		auto scroll = ea.getScrollingMotion();
		switch(scroll)
		{
		case(osgGA::GUIEventAdapter::NONE):
		case(osgGA::GUIEventAdapter::SCROLL_2D):
		{
			break;
		}
		case(osgGA::GUIEventAdapter::SCROLL_UP):
		{
			m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, 1);
			break;
		}
		case(osgGA::GUIEventAdapter::SCROLL_DOWN):
		{
			m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, -1);
			break;
		}
		case(osgGA::GUIEventAdapter::SCROLL_LEFT):
		{	m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), -1, 0);
			break;
		}
		case(osgGA::GUIEventAdapter::SCROLL_RIGHT):
		{	m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 1, 0);
			break;
		}
		}
		lastEvent = osgGA::GUIEventAdapter::SCROLL;
		handled = true;
		break;
	}
	case(osgGA::GUIEventAdapter::DRAG) :
	{
		m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, 0);
		lastEvent = osgGA::GUIEventAdapter::DRAG;
		handled = true;
		break;
	}
	case(osgGA::GUIEventAdapter::PUSH) :
	{
		m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, 0);
		lastEvent = osgGA::GUIEventAdapter::PUSH;
		handled = true;
		break;
	}
	case(osgGA::GUIEventAdapter::MOVE) :
	{
		if (osgGA::GUIEventAdapter::EventType::RELEASE != lastEvent && ea.getX() != lastX && ea.getY() != lastY)
		{
			m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, 0);
		}
		lastEvent = osgGA::GUIEventAdapter::MOVE;
		handled = true;
		break;
	}
	case(osgGA::GUIEventAdapter::RELEASE) :
	{
		m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, 0);
		lastEvent = osgGA::GUIEventAdapter::RELEASE;
		handled = true;
		break;
	}
	}

	lastX = ea.getX();
	lastY = ea.getY();

	return handled;
}

};  // namespace Device
};  // namespace SurgSim

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
			handled = true;
			break;
		}
		case(osgGA::GUIEventAdapter::SCROLL_DOWN):
		{
			m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, -1);
			handled = true;
			break;
		}
		case(osgGA::GUIEventAdapter::SCROLL_LEFT):
		{	m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), -1, 0);
			handled = true;
			break;
		}
		case(osgGA::GUIEventAdapter::SCROLL_RIGHT):
		{	m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 1, 0);
			handled = true;
			break;
		}
		default:
			break;
		}
		break;
	}
	case(osgGA::GUIEventAdapter::MOVE) :
	{// Work around for the issue that osgGA generates an extra "MOVE" event after a "RELEASE" event on Windows platform
		if (! (osgGA::GUIEventAdapter::EventType::RELEASE == m_lastEvent &&
			   ea.getX() == m_lastX && ea.getY() == m_lastY))
		{
			m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, 0);
		}
		handled = true;
		break;
	}
	case(osgGA::GUIEventAdapter::DRAG) :
	case(osgGA::GUIEventAdapter::PUSH) :
	case(osgGA::GUIEventAdapter::RELEASE) :
	{
		m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, 0);
		handled = true;
		break;
	}
	default:
		break;
	}

	m_lastEvent = eventType;
	m_lastX = ea.getX();
	m_lastY = ea.getY();

	return handled;
}

};  // namespace Device
};  // namespace SurgSim

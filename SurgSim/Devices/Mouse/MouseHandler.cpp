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

MouseHandler::MouseHandler() : m_mouseScaffold(MouseScaffold::getOrCreateSharedInstance()),
							   m_lastX(0.0), m_lastY(0.0), m_lastButtonMask(0), m_lastScrollX(0), m_lastScrollY(0)
{
}

bool MouseHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
	int scrollX = 0, scrollY = 0;
	if (ea.getEventType() == osgGA::GUIEventAdapter::EventType::SCROLL)
	{
		switch(ea.getScrollingMotion())
		{
		case(osgGA::GUIEventAdapter::SCROLL_UP) :
		{
			scrollY = 1;
			break;
		}
		case(osgGA::GUIEventAdapter::SCROLL_DOWN) :
		{
			scrollY = -1;
			break;
		}
		case(osgGA::GUIEventAdapter::SCROLL_LEFT) :
		{
			scrollX = -1;
			break;
		}
		case(osgGA::GUIEventAdapter::SCROLL_RIGHT) :
		{
			scrollX = 1;
			break;
		}
		default:
			break;
		}
	}

	// Any mouse state change will cause an update
	if( ea.getX() != m_lastX || ea.getY() != m_lastY ||
		ea.getButtonMask() != m_lastButtonMask ||
		m_lastScrollX != scrollX || m_lastScrollY != scrollY)
	{
		m_lastX = ea.getX();
		m_lastY = ea.getY();
		m_lastButtonMask = ea.getButtonMask();
		m_lastScrollX = scrollX;
		m_lastScrollY = scrollY;

		m_mouseScaffold.lock()->updateDevice(m_lastButtonMask, m_lastX, m_lastY, m_lastScrollX, m_lastScrollY);
	}

	return true;
}

};  // namespace Device
};  // namespace SurgSim

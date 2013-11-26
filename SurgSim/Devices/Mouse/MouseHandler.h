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

#ifndef SURGSIM_DEVICES_MOUSE_MOUSEHANDLER_H
#define SURGSIM_DEVICES_MOUSE_MOUSEHANDLER_H

#include <SurgSim/Devices/Mouse/MouseScaffold.h>

#include <memory>

#include <osgGA/GUIEventHandler>

namespace SurgSim
{
namespace Device
{

class MouseHandler : public osgGA::GUIEventHandler
{
public:
	/// Constructor
	MouseHandler() : m_mouseScaffold(MouseScaffold::getOrCreateSharedInstance())
	{
	}

	/// Method to handle GUI event
	/// \param ea A osgGA::GUIEventAdapter
	/// \param _2 A dummy parameter (required by this virtual method)
	/// \return True if the event has been handled by this method; Otherwise, false.
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override
	{
		switch(ea.getEventType())
		{
		case(osgGA::GUIEventAdapter::SCROLL) :
		{
			auto scroll = ea.getScrollingMotion();
			switch(scroll)
			{
			case(osgGA::GUIEventAdapter::SCROLL_UP):
				m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, 1);
				break;
			case(osgGA::GUIEventAdapter::SCROLL_DOWN):
				m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, -1);
				break;
			case(osgGA::GUIEventAdapter::SCROLL_LEFT):
				m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), -1, 0);
				break;
			case(osgGA::GUIEventAdapter::SCROLL_RIGHT):
				m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 1, 0);
				break;
			}
			return true;
		}
		case(osgGA::GUIEventAdapter::MOVE) :
		case(osgGA::GUIEventAdapter::DRAG) :
		case(osgGA::GUIEventAdapter::PUSH) :
		case(osgGA::GUIEventAdapter::RELEASE) :
		{
			m_mouseScaffold.lock()->updateDevice(ea.getButtonMask(), ea.getX(), ea.getY(), 0, 0);
			return true;
		}
		default:
			return false;
		}
	}

private:
	/// A back pointer to the scaffold which owns this handle
	std::weak_ptr<MouseScaffold> m_mouseScaffold;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MOUSE_MOUSEHANDLER_H
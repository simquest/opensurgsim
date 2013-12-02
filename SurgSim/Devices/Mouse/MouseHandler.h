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

#include <memory>

#include <osgGA/GUIEventHandler>

namespace SurgSim
{
namespace Device
{

class MouseScaffold;

class MouseHandler : public osgGA::GUIEventHandler
{
public:
	/// Constructor
	MouseHandler();

	/// Method to handle GUI event
	/// \param ea A osgGA::GUIEventAdapter
	/// \param _2 A dummy parameter (required by this virtual method)
	/// \return True if the event has been handled by this method; Otherwise, false.
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override;

private:
	/// A back pointer to the scaffold which owns this handle
	std::weak_ptr<MouseScaffold> m_mouseScaffold;

	/// \note The following variables are work around for the issue that OSG will generates an extra "MOVE" event after
	/// mouse button release on Windows platform.

	/// Used to record last mouse event
	osgGA::GUIEventAdapter::EventType m_lastEvent;
	/// lastX is the X-coordinate of mouse's last location
	/// lastY is the Y-coordinate of mouse's last location
	float m_lastX, m_lastY;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MOUSE_MOUSEHANDLER_H
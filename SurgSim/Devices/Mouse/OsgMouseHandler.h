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

#ifndef SURGSIM_DEVICES_MOUSE_OSGMOUSEHANDLER_H
#define SURGSIM_DEVICES_MOUSE_OSGMOUSEHANDLER_H

#include <memory>

#include <osgGA/GUIEventHandler>

namespace SurgSim
{
namespace Device
{

class MouseScaffold;

class OsgMouseHandler : public osgGA::GUIEventHandler
{
public:
	/// Constructor
	OsgMouseHandler();

	/// Method to handle GUI event
	/// \param eventHandler A osgGA::GUIEventAdapter
	/// \param actionAdapter A osgGA::GUIActionAdapter (required by this virtual method)
	/// \return True if the event has been handled by this method; Otherwise, false.
	bool handle(const osgGA::GUIEventAdapter& eventHandler, osgGA::GUIActionAdapter& actionAdapter) override;

private:
	/// A back pointer to the scaffold which owns this handle
	std::weak_ptr<MouseScaffold> m_mouseScaffold;

	/// lastX is the X-coordinate of mouse's last location
	/// lastY is the Y-coordinate of mouse's last location
	float m_lastX, m_lastY;

	/// Last button mask
	int m_lastButtonMask;
	/// Last direction of mouse wheel's horizontal (X) and vertical (Y) movement.
	int m_lastScrollX, m_lastScrollY;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MOUSE_OSGMOUSEHANDLER_H
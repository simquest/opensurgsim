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

#ifndef SURGSIM_DEVICES_MOUSE_MOUSEDEVICE_H
#define SURGSIM_DEVICES_MOUSE_MOUSEDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Device
{

class MouseScaffold;
class MouseHandler;

/// A class implementing the communication with a mouse
///
/// \par Application input provided from the device:
///   | type       | name              |                                                          |
///   | ----       | ----              | ---                                                      |
///   | bool       | "button1"         | %State of mouse left button						      |
///   | bool       | "button2"		   | %State of mouse middle button							  |
///   | bool       | "button3"		   | %State of mouse right button							  |
///   | scalar	   | "x"			   | %X-coordinate of mouse									  |
///   | scalar	   | "y"			   | %Y-coordinate of mouse									  |
///   | int		   | "scrollDeltaX"	   | %Indicates vertical movement direction of mouse wheel	  |
///   | int		   | "scrollDeltaY"	   | %Indicates horizontal movement direction of mouse wheel  |
///
/// \par Application output used by the device:
///      NONE
///
/// \note  osgGA will generate scroll events when mouse wheel starts to move (up/down/left/right) and store vertical
///        movements (+1 for scroll up and -1 for scroll down) in 'scrollDeltaX" and horizontal movements (-1 for scroll
///        left and +1 for scroll right) in 'scrollDeltaY".
///        However, osgGA will not generate any event when the movements stop.
///        Values of 'scrollDeltaX/Y" will be set to 0 (indicating no movement) in the next (non wheel) mouse event.
///
/// \sa SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class MouseDevice : public SurgSim::Input::CommonDevice
{
	friend class MouseScaffold;
	friend class MouseDeviceTest;

public:
	/// Constructor
	/// \param deviceName Name for mouse device
	explicit MouseDevice(const std::string& deviceName);
	/// Destructor
	virtual ~MouseDevice();

	/// Initialize corresponding MouseScaffold.
	/// \return True if MouseScaffold is initialized successfully; Otherwise, false.
	virtual bool initialize() override;
	/// "De"-initialize corresponding MouseScaffold.
	/// \return True if MouseScaffold is 'de'-initialized successfully; Otherwise, false.
	virtual bool finalize() override;

	/// Check if the scaffold of this device is initialized.
	/// \return True if this the scaffold of this device is initialized; Otherwise, false.
	bool isInitialized() const;

	/// Get mouse handler
	/// \return The mouse handler associated with this device
	MouseHandler* getMouseHandler() const;

private:
	/// Communication with hardware is handled by scaffold.
	std::shared_ptr<MouseScaffold> m_scaffold;
};

};  // namespace Device
};  // namespace SurgSim

#endif //SURGSIM_DEVICES_MOUSE_MOUSEDEVICE_H
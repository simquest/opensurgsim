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

#include "SurgSim/Input/CommonDevice.h"

#include <memory>
#include <string>

namespace SurgSim
{
namespace Devices
{
class MouseScaffold;
class OsgMouseHandler;

SURGSIM_STATIC_REGISTRATION(MouseDevice);

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
/// \note  Mouse wheel movement will be indicated by +1/-1 (scroll up/down, right/left) followed by a 0.
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

	SURGSIM_CLASSNAME(SurgSim::Devices::MouseDevice);

	/// Destructor
	virtual ~MouseDevice();

	bool initialize() override;

	bool isInitialized() const override;

	/// Get mouse handler
	/// \return The mouse handler associated with this device
	OsgMouseHandler* getMouseHandler() const;

private:
	bool finalize() override;

	/// Communication with hardware is handled by scaffold.
	std::shared_ptr<MouseScaffold> m_scaffold;
};

};  // namespace Devices
};  // namespace SurgSim

#endif //SURGSIM_DEVICES_MOUSE_MOUSEDEVICE_H
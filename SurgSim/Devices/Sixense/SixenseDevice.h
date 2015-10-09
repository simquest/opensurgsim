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

#ifndef SURGSIM_DEVICES_SIXENSE_SIXENSEDEVICE_H
#define SURGSIM_DEVICES_SIXENSE_SIXENSEDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Devices
{
class SixenseScaffold;

SURGSIM_STATIC_REGISTRATION(SixenseDevice);

/// A class implementing the communication with one Sixense controller, for example one of the two on the Razer Hydra.
///
/// \par Application input provided by the device:
///   | type       | name              |                                                                           |
///   | ----       | ----              | ---                                                                       |
///   | pose       | "pose"            | %Device pose (units are meters).                                          |
///   | scalar     | "trigger"         | State of the analog trigger button (0 = not pressed, 1 = fully pressed). |
///   | scalar     | "joystickX"       | Joystick X position (0 = center, -1 = fully left, +1 = fully right).      |
///   | scalar     | "joystickY"       | Joystick Y position (0 = center, -1 = fully down/near, +1 = up/far).      |
///   | bool       | "buttonTrigger"   | True if the analog trigger button is pressed, i.e. its value is non-zero. |
///   | bool       | "buttonBumper"    | True if the bumper button (next to the trigger) is pressed.               |
///   | bool       | "button1"         | True if the button marked "1" is pressed.                                 |
///   | bool       | "button2"         | True if the button marked "2" is pressed.                                 |
///   | bool       | "button3"         | True if the button marked "3" is pressed.                                 |
///   | bool       | "button4"         | True if the button marked "4" is pressed.                                 |
///   | bool       | "buttonStart"     | True if the "start" button is pressed.                                    |
///   | bool       | "buttonJoystick"  | True if the joystick is pressed down as a button ("into" the controller). |
///
/// \par Application output used by the device: none.
///
/// \sa SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class SixenseDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	explicit SixenseDevice(const std::string& uniqueName);

	SURGSIM_CLASSNAME(SurgSim::Devices::SixenseDevice);

	/// Destructor.
	virtual ~SixenseDevice();

	bool initialize() override;

	bool isInitialized() const override;

private:
	friend class SixenseScaffold;

	bool finalize() override;

	std::shared_ptr<SixenseScaffold> m_scaffold;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_SIXENSE_SIXENSEDEVICE_H

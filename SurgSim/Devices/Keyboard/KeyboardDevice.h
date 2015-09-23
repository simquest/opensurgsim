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

#ifndef SURGSIM_DEVICES_KEYBOARD_KEYBOARDDEVICE_H
#define SURGSIM_DEVICES_KEYBOARD_KEYBOARDDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Devices
{
class KeyboardScaffold;
class OsgKeyboardHandler;

SURGSIM_STATIC_REGISTRATION(KeyboardDevice);

/// A class implementing the communication with a keyboard
///
/// \par Application input provided from the device:
///   | type       | name			  |															  |
///   | ----       | ----			  | ---                                      				  |
///   | int		   | "key"			  | %Key code for the pressed key, if any. Default: -1		  |
///   | int		   | "modifierMask"   | %Mask for the pressed modifier, if any. Default: 0        |
///
/// \par Application output used by the device:
///	  NONE
///
/// \note Key 'Fn' is not currently captured (No key code is assigned to it).
/// \sa SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class KeyboardDevice : public SurgSim::Input::CommonDevice
{
	friend class KeyboardScaffold;
	friend class KeyboardDeviceTest;

public:
	/// Constructor
	/// \param deviceName Name for keyboard device
	explicit KeyboardDevice(const std::string& deviceName);

	SURGSIM_CLASSNAME(SurgSim::Devices::KeyboardDevice);

	/// Destructor
	virtual ~KeyboardDevice();

	bool initialize() override;

	bool finalize() override;

	/// Check if the scaffold of this device is initialized.
	/// \return True if this the scaffold of this device is initialized; Otherwise, false.
	bool isInitialized() const;

	/// Get keyboard handler
	/// \return The keyboard handler associated with this device
	OsgKeyboardHandler* getKeyboardHandler() const;

private:
	/// Communication with hardware is handled by scaffold.
	std::shared_ptr<KeyboardScaffold> m_scaffold;
};

};  // namespace Devices
};  // namespace SurgSim

#endif //SURGSIM_DEVICES_KEYBOARD_KEYBOARDDEVICE_H
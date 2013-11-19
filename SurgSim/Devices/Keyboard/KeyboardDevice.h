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

#include <SurgSim/Input/CommonDevice.h>

namespace SurgSim
{
namespace Device
{
class KeyboardScaffold;
class KeyboardHandler;

/// A class implementing the communication with a keyboard
class KeyboardDevice : public SurgSim::Input::CommonDevice
{
	friend class KeyboardScaffold;
	friend class KeyboardDeviceTest;

public:
	/// Constructor
	/// \param deviceName Name for keyboard device
	explicit KeyboardDevice(const std::string& deviceName);
	/// Destructor
	virtual ~KeyboardDevice();

	/// Initialize corresponding KeyboardScaffold.
	/// \return True if KeyboardScaffold is intialized successfully; Otherwise, false.
	virtual bool initialize() override;
	/// "De"-initialize corresponding KeyboardScaffold.
	/// \return True if KeyboardScaffold is 'de'-intialized successfully; Otherwise, false.
	virtual bool finalize() override;

	/// Check if the scaffold of this device is initialized.
	/// \return True if this the scaffold of this device is initialized; Otherwise, false.
	bool isInitialized() const;

	/// Get keyboard handler
	/// \return The keyboard handler associated with this device
	std::shared_ptr<KeyboardHandler> getKeyboardHandler() const;

private:
	/// Communication with hardware is handled by scaffold.
	std::shared_ptr<KeyboardScaffold> m_scaffold;
};

};  // namespace Device
};  // namespace SurgSim

#endif //SURGSIM_DEVICES_KEYBOARD_KEYBOARDDEVICE_H
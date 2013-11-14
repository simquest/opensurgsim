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

/// A class implementing the communication with a keyboard
class KeyboardDevice : public SurgSim::Input::CommonDevice
{
	friend class KeyboardScaffold;

public:
	/// Constructor
	/// \param uniqueName A unique name for the device that will be used by the application.
	explicit KeyboardDevice(const std::string& deviceName);
	/// Destructor
	virtual ~KeyboardDevice();

	/// Initialize corresponding KeyboardScaffold
	/// \return True if KeyboardScaffold intialized successfully; Otherwise, false
	virtual bool initialize() override;
	/// "De"-initialize corresponding KeyboardScaffold
	/// \return True if KeyboardScaffold 'de'-intialized successfully; Otherwise, false
	virtual bool finalize() override;

	/// Check if the scaffold of this device is initialized.
	/// \return True if this the scaffold of this device is initialized; Otherwise, false
	bool isInitialized() const;

	/// Sets the delay before repeat for this device.
	/// The delay before repeat controls when a pressed key starts to repeat itself.
	/// \param delay Time before repeat starts
	void setDelay(double delay);
	/// Gets the delay before repeat for this device
	/// \return Delay before repeat
	double getDelay() const;

	/// Sets the repeat speed for this device.
	/// The repeat speed controls how fast a pressed key repeats itself.
	/// \param repeatSpeed Speed of repeat
	void setRepeatSpeed(double repeatSpeed);
	/// Gets the speed of repeat of a key
	/// \return Speed of repeat
	double getRepeatSpeed() const;

private:
	/// Communication with hardware is handled by scaffold
	std::shared_ptr<KeyboardScaffold> m_scaffold;

	/// Time before a pressed key starts to repeat itself
	double m_delay;

	/// Speed of a key repeating itself
	double m_repeatSpeed;

};

};  // namespace Device
};  // namespace SurgSim

#endif //SURGSIM_DEVICES_KEYBOARD_KEYBOARDDEVICE_H
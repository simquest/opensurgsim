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

#ifndef SURGSIM_DEVICE_SIXENSE_DEVICE_H
#define SURGSIM_DEVICE_SIXENSE_DEVICE_H

#include <memory>
#include <string>

#include <SurgSim/Input/CommonDevice.h>

namespace SurgSim
{
namespace Device
{

class SixenseManager;

/// A class implementing the communication with one Sixense controller, for example one of the two on the Razer Hydra.
///
/// \par Application input provided by the device:
///   | type       | name              |                                                                           |
///   | ----       | ----              | ---                                                                       |
///   | pose       | "pose"            | %Device pose (units are meters).                                          |
///   | scalar     | "trigger"         | %State of the analog trigger button (0 = not pressed, 1 = fully pressed). |
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
/// \sa SurgSim::Device::SixenseManager, SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class SixenseDevice : public SurgSim::Input::CommonDevice
{
public:
	virtual ~SixenseDevice();

	/// Gets the logger used by this object and the devices it manages.
	/// \return The logger.
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const
	{
		return m_logger;
	}

protected:
	friend class SixenseManager;

	/// Constructor.
	///
	/// \param manager The SixenseManager object creating the device.
	/// \param uniqueName A unique name for the device that will be used by the application.
	/// \param baseIndex Index of the base unit this controller is connected to.
	/// \param controllerIndex Index of this controller within its base unit.
	SixenseDevice(const SixenseManager& manager, const std::string& uniqueName, int baseIndex, int controllerIndex);

	/// Gets the index of the base unit this controller is connected to.
	/// \return The base unit index.
	int getBaseIndex() const
	{
		return m_baseIndex;
	}

	/// Gets the index of this controller within its base unit.
	/// \return The controller index.
	int getControllerIndex() const
	{
		return m_controllerIndex;
	}

	virtual bool initialize();

	virtual bool finalize();

	/// Communicates with the device, reading its state and writing the command parameters.
	/// \return true on success.
	bool update();

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildInputData();

private:
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
	int m_baseIndex;
	int m_controllerIndex;
	std::string m_messageLabel;
};

};  // namespace Device
};  // namespace SurgSim

#endif // SURGSIM_DEVICE_SIXENSE_DEVICE_H

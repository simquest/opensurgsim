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

#ifndef SURGSIM_DEVICES_PHANTOM_PHANTOMDEVICE_H
#define SURGSIM_DEVICES_PHANTOM_PHANTOMDEVICE_H

#include <memory>
#include <string>

#include <SurgSim/Input/CommonDevice.h>

namespace SurgSim
{
namespace Device
{

class PhantomManager;

/// A class implementing the communication with a Sensable PHANTOM device.
///
/// This should support any PHANTOM device that can communicate using OpenHaptics 3.0 toolkit, such as PHANTOM
/// Omni, PHANTOM Desktop, and the PHANTOM Premium series devices.  The implementation is currently limited to
/// 3DoF haptic output (forces only, no torques).
///
/// \par Application input provided by the device:
///   | type       | name        |                                                                |
///   | ----       | ----        | ---                                                            |
///   | pose       | "pose"      | %Device pose (units are meters).                               |
///   | bool       | "button0"   | %State of the first device button.                             |
///   | bool       | "button1"   | %State of the second device button if present.                 |
///   | bool       | "button2"   | %State of the third device button (probaly doesn't exist).     |
///   | bool       | "button3"   | %State of the third device button (probaly doesn't exist).     |
/// Note that \c button0 through \c 3 correspond to the \c HD_DEVICE_BUTTON_1 through \c 4 provided by the
/// OpenHaptics API, but your PHANTOM device likely has fewer than 4 buttons.  On one-button PHANTOM devices,
/// the button state can be accessed through \c button0.  On a PHANTOM Omni, \c button0
/// corresponds to the front (blue) stylus button, and \c button1 to the rear (white/gray) stylus button.
///
/// \par Application output used by the device:
///   | type       | name        |                                                                |
///   | ----       | ----        | ---                                                            |
///   | vector     | "force"     | %Device output force (units are newtons).                      |
///   | vector     | "torque"    | %Device output torque (units are Nm).  NOT YET SUPPORTED.      |
///
/// \sa SurgSim::Device::PhantomManager, SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class PhantomDevice : public SurgSim::Input::CommonDevice
{
public:
	virtual ~PhantomDevice();

	/// Gets the logger used by this object and the devices it manages.
	/// \return The logger.
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const
	{
		return m_logger;
	}

protected:
	/// Constructor.
	///
	/// \param manager The PhantomManager object creating the device.
	/// \param uniqueName A unique name for the device that will be used by the application.
	/// \param initializationName The name passed to HDAPI when initializing the device.  This should match a
	/// 	configured PHANTOM device; alternately, an empty string indicates the default device.
	PhantomDevice(const PhantomManager& manager, const std::string& uniqueName, const std::string& initializationName);

	friend class PhantomManager;

	virtual bool initialize();

	virtual bool finalize();

	/// Communicates with the device, reading its state and writing the command parameters.
	/// \return true on success.
	bool update();

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildInputData();

	/// Check for OpenHaptics HDAPI errors, display them, and signal fatal errors.
	///
	/// \param message An additional descriptive message.
	/// \return true if there was a fatal error; false if everything is OK.
	bool checkForFatalError(const char* message);

	/// Check for OpenHaptics HDAPI errors, display them, and signal fatal errors.
	/// This is an internal version of the method that is used by PhantomManager as well as PhantomDevice.
	///
	/// \param logger The logger to be used.
	/// \param prefix A prefix to be shown before the \a message (e.g. "Manager: "), or an empty string.
	/// \param message An additional descriptive message.
	/// \return true if there was a fatal error; false if everything is OK.
	static bool checkForFatalError(const std::shared_ptr<SurgSim::Framework::Logger>& logger,
	                               const char* prefix, const char* message);

private:
	/// Internal device state.
	struct State;

	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
	std::string m_initializationName;
	std::string m_messageLabel;
	std::unique_ptr<State> m_state;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_PHANTOM_PHANTOMDEVICE_H

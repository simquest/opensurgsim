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

#ifndef SURGSIM_DEVICE_PHANTOM_DEVICE_H
#define SURGSIM_DEVICE_PHANTOM_DEVICE_H

#include <memory>
#include <string>

#include <SurgSim/Input/CommonInputDevice.h>

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
/// \sa SurgSim::Input::InputDeviceInterface
class PhantomDevice : public SurgSim::Input::CommonInputDevice
{
public:
	virtual ~PhantomDevice();

	/// Gets the logger used by this object and the devices it manages.
	/// \return The logger.
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const
	{
		return m_logger;
	}

	/// Communicates with the device, reading its state and writing the command parameters.
	/// \return true on success.
	bool update();

protected:
	/// Constructor.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	/// \param initializationName The name passed to HDAPI when initializing the device.  This should match a
	/// 	configured PHANTOM device; alternately, an empty string indicates the default device.
	PhantomDevice(const PhantomManager& manager, const std::string& uniqueName, const std::string& initializationName);

	friend class PhantomManager;

	virtual bool initialize();

	virtual bool finalize();

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::Input::DataGroup buildInputData();

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
	State* m_state;
};

};  // namespace Device
};  // namespace SurgSim

#endif // SURGSIM_DEVICE_PHANTOM_DEVICE_H

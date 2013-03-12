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

#ifndef SURGSIM_DEVICE_PHANTOM_MANAGER_H
#define SURGSIM_DEVICE_PHANTOM_MANAGER_H

#include <memory>

#include <SurgSim/Framework/Logger.h>

namespace SurgSim
{
namespace Device
{

class PhantomDevice;

/// A class that manages Sensable PHANTOM devices.
///
/// This should support any PHANTOM device that can communicate using OpenHaptics 3.0 toolkit, such as PHANTOM
/// Omni, PHANTOM Desktop, and the PHANTOM Premium series devices.  The implementation is currently limited to
/// 3DoF haptic output (forces only, no torques).
///
/// \sa SurgSim::Device::PhantomDevice
class PhantomManager
{
public:
	/// Constructor.
	/// \param logger (optional) The logger to be used for the manager object and the devices it manages.
	/// 			  If unspecified or empty, a console logger will be created and used.
	explicit PhantomManager(std::shared_ptr<SurgSim::Framework::Logger> logger =
	                            std::shared_ptr<SurgSim::Framework::Logger>());

	/// Destructor.
	~PhantomManager();

	/// Gets the logger used by this object and the devices it manages.
	/// \return The logger.
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const
	{
		return m_logger;
	}

	/// Creates a device.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	/// \param initializationName The name passed to HDAPI when initializing the device.  This should match a
	/// 	configured PHANTOM device; alternately, an empty string indicates the default device.
	/// \return The newly created device, or an empty shared_ptr if the initialization fails.
	std::shared_ptr<PhantomDevice> createDevice(const std::string& uniqueName, const std::string& initializationName);

	/// Shuts down and releases (and possibly destroys) the specified device.
	///
	/// After being released, the device object will be destroyed, but only if there are no other shared_ptr
	/// references to it being held elsewhere.
	///
	/// \param device The device.
	/// \return true on success, false on failure.
	bool releaseDevice(std::shared_ptr<PhantomDevice> device);

	/// Executes the operations for a single haptic frame.
	/// Should only be called from the context of an OpenHaptics callback.
	/// \return true on success.
	bool runHapticFrame();

protected:
	/// Creates the haptic loop callback.
	/// \return true on success.
	bool createHapticLoop();

	/// Destroys the haptic loop callback.
	/// \return true on success.
	bool destroyHapticLoop();

	/// Check for OpenHaptics HDAPI errors, display them, and signal fatal errors.
	///
	/// \param message An additional descriptive message.
	/// \return true if there was a fatal error; false if everything is OK.
	bool checkForFatalError(const char* message);

private:
	/// Internal manager state.
	struct State;

	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
	std::unique_ptr<State> m_state;
};

};  // namespace Device
};  // namespace SurgSim

#endif // SURGSIM_DEVICE_PHANTOM_MANAGER_H

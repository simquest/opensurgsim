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

#ifndef SURGSIM_DEVICE_SIXENSEMANAGER_H
#define SURGSIM_DEVICE_SIXENSEMANAGER_H

#include <memory>

#include <SurgSim/Framework/Logger.h>

namespace SurgSim
{
namespace Device
{

class SixenseDevice;
class SixenseThread;

/// A class that manages Sixense devices, such as the Razer Hydra.
///
/// \sa SurgSim::Device::SixenseDevice
class SixenseManager
{
public:
	/// Constructor.
	/// \param logger (optional) The logger to be used for the manager object and the devices it manages.
	/// 			  If unspecified or empty, a console logger will be created and used.
	explicit SixenseManager(std::shared_ptr<SurgSim::Framework::Logger> logger =
	                            std::shared_ptr<SurgSim::Framework::Logger>());

	/// Destructor.
	~SixenseManager();

	/// Gets the logger used by this object and the devices it manages.
	/// \return The logger.
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const
	{
		return m_logger;
	}

	/// Sets the default log level.
	/// Must be called before a manager is created (i.e. before the first device) in order to have any effect.
	/// \param logLevel The log level.
	static void setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel);

protected:
	friend class SixenseDevice;
	friend class SixenseThread;

	/// Creates a device.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	/// \return The newly created device, or an empty shared_ptr if the initialization fails.
	std::shared_ptr<SixenseDevice> createDevice(const std::string& uniqueName);

	/// Shuts down and releases (and possibly destroys) the specified device.
	///
	/// After being released, the device object will be destroyed, but only if there are no other shared_ptr
	/// references to it being held elsewhere.
	///
	/// \param device The device.
	/// \return true on success, false on failure.
	bool releaseDevice(const SixenseDevice* device);

	/// Executes the operations for a single input frame.
	/// Should only be called from the context of the input loop thread.
	/// \return true on success.
	bool runInputFrame();

	/// Initializes the Sixense SDK.
	/// \return true on success.
	bool initializeSdk();

	/// Finalizes (de-initializes) the Sixense SDK.
	/// \return true on success.
	bool finalizeSdk();

	/// Scans controllers that are present in the system, and if an unused one is found, create a device for it.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	/// \param [out] device The created device handle, if any.
	/// \param [out] numUsedDevicesSeen The number of devices that were found during the scan but were already
	/// 	in use.  Can be used if the scan fails to determine the error message that should be displayed.
	///
	/// \return true on success.
	bool scanForUnusedDevice(const std::string& uniqueName, std::shared_ptr<SixenseDevice>* device,
	                         int* numUsedDevicesSeen);

	/// Creates a device object given a (baseIndex, controllerIndex) pair, if the same pair is not already in use.
	///
	/// \param baseIndex Index of the base unit.
	/// \param controllerIndex Index of the controller within the base unit.
	/// \param uniqueName A unique name for the created device that will be used by the application.
	/// \param [out] device The created device handle, if any.
	/// \param [in,out] numUsedDevicesSeen The number of devices that were found during the scan but were
	/// 	already in use; incremented when an unused device is seen.
	///
	/// \return true on success.
	bool createDeviceIfUnused(int baseIndex, int controllerIndex, const std::string& uniqueName,
	                          std::shared_ptr<SixenseDevice>* device, int* numUsedDevicesSeen);

	/// Creates the input loop thread.
	/// \return true on success.
	bool createThread();

	/// Destroys the input loop thread.
	/// Should be called while NOT holding the internal device list mutex, to prevent deadlock.
	/// \return true on success.
	bool destroyThread();

private:
	/// Internal manager state data type.
	struct State;

	/// Logger used by the manager and all devices.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
	/// Internal manager state.
	std::unique_ptr<State> m_state;

	/// The default logging level.
	static SurgSim::Framework::LogLevel m_defaultLogLevel;
	/// How long we're willing to wait for devices to be detected, in milliseconds.
	static int m_startupDelayMilliseconds;
};

};  // namespace Device
};  // namespace SurgSim

#endif // SURGSIM_DEVICE_SIXENSEMANAGER_H

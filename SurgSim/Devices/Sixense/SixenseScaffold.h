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

#ifndef SURGSIM_DEVICES_SIXENSE_SIXENSESCAFFOLD_H
#define SURGSIM_DEVICES_SIXENSE_SIXENSESCAFFOLD_H

#include <memory>

#include "SurgSim/Framework/Logger.h"
#include "SurgSim/DataStructures/DataGroup.h"

namespace SurgSim
{
namespace Device
{

class SixenseDevice;
class SixenseThread;

/// A class that manages Sixense devices, such as the Razer Hydra.
///
/// \sa SurgSim::Device::SixenseDevice
class SixenseScaffold
{
public:
	/// Destructor.
	~SixenseScaffold();

	/// Gets the logger used by this object and the devices it manages.
	/// \return The logger.
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const
	{
		return m_logger;
	}

	/// Gets or creates the scaffold shared by all SixenseDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<SixenseScaffold> getOrCreateSharedInstance();

	/// Sets the default log level.
	/// Has no effect unless called before a scaffold is created (i.e. before the first device).
	/// \param logLevel The log level.
	static void setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel);

private:
	/// Internal shared state data type.
	struct StateData;
	/// Interal per-device information.
	struct DeviceData;

	friend class SixenseDevice;
	friend class SixenseThread;
	friend struct StateData;

	/// Constructor.
	/// \param logger (optional) The logger to be used for the scaffold object and the devices it manages.
	/// 			  If unspecified or empty, a console logger will be created and used.
	explicit SixenseScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger = nullptr);

	/// Registers the specified device object.
	/// If successful, the device object will become connected to an unused controller.
	///
	/// \param device The device object to be used, which should have a unique name.
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(SixenseDevice* device);

	/// Unregisters the specified device object.
	/// The corresponding controller will become unused, and can be re-registered later.
	///
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const SixenseDevice* device);

	/// Executes the operations for a single input frame.
	/// Should only be called from the context of the input loop thread.
	/// \return true on success.
	bool runInputFrame();

	/// Updates the device information for a single device.
	/// \return true on success.
	bool updateDevice(const DeviceData& info);

	/// Initializes the Sixense SDK.
	/// \return true on success.
	bool initializeSdk();

	/// Finalizes (de-initializes) the Sixense SDK.
	/// \return true on success.
	bool finalizeSdk();

	/// Scans controllers that are present in the system, and if an unused one is found, register a device for it.
	///
	/// \param device The device object to register if an unused device is found.
	/// \param [out] numUsedDevicesSeen The number of devices that were found during the scan but were already
	/// 	in use.  Can be used if the scan fails to determine the error message that should be displayed.
	/// \param [out] fatalError Set to true if an error (such as a duplicate device name) prevented device
	/// 	registration such that retrying will not help; false otherwise.
	/// \return true on success.
	bool findUnusedDeviceAndRegister(SixenseDevice* device, int* numUsedDevicesSeen, bool* fatalError);

	/// Register a device object given a (baseIndex, controllerIndex) pair, if the same pair is not already in use.
	///
	/// \param baseIndex Index of the base unit.
	/// \param controllerIndex Index of the controller within the base unit.
	/// \param device The device object to register if the index pair is in fact unused.
	/// \param [in,out] numUsedDevicesSeen The number of devices that were found during the scan but were
	/// 	already in use; incremented when an unused device is seen.
	/// \return true on success.
	bool registerIfUnused(int baseIndex, int controllerIndex, SixenseDevice* device, int* numUsedDevicesSeen);

	/// Creates the input loop thread.
	/// \return true on success.
	bool createThread();

	/// Destroys the input loop thread.
	/// Should be called while NOT holding the internal device list mutex, to prevent deadlock.
	/// \return true on success.
	bool destroyThread();

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();



	/// Logger used by the scaffold and all devices.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;

	/// The default logging level.
	static SurgSim::Framework::LogLevel m_defaultLogLevel;
	/// How long we're willing to wait for devices to be detected, in milliseconds.
	static int m_startupDelayMilliseconds;
	/// How long to wait between trying to detect devices, in milliseconds.
	static int m_startupRetryIntervalMilliseconds;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_SIXENSE_SIXENSESCAFFOLD_H

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

#ifndef SURGSIM_DEVICES_MULTIAXIS_RAWMULTIAXISSCAFFOLD_H
#define SURGSIM_DEVICES_MULTIAXIS_RAWMULTIAXISSCAFFOLD_H

#include <memory>
#include <vector>

#include "SurgSim/Framework/Logger.h"
#include "SurgSim/DataStructures/DataGroup.h"

namespace SurgSim
{
namespace Device
{

class RawMultiAxisDevice;
class RawMultiAxisThread;
class SystemInputDeviceHandle;

/// A class that implements the behavior of RawMultiAxisDevice objects.
///
/// \sa SurgSim::Device::RawMultiAxisDevice
class RawMultiAxisScaffold
{
public:
	/// Constructor.
	RawMultiAxisScaffold();

	/// Destructor.
	~RawMultiAxisScaffold();

	/// Gets the logger used by this object and the devices it manages.
	/// \return The logger.
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const;

	/// Gets or creates the scaffold shared by all RawMultiAxisDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<RawMultiAxisScaffold> getOrCreateSharedInstance();

	/// Sets the default log level.
	/// Has no effect unless called before a scaffold is created (i.e. before the first device).
	/// \param logLevel The log level.
	static void setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel);

private:
	/// Internal shared state data type.
	struct StateData;
	/// Interal per-device information.
	struct DeviceData;

	friend class RawMultiAxisDevice;
	friend class RawMultiAxisThread;
	friend struct StateData;

	/// Registers the specified device object.
	/// If successful, the device object will become connected to an unused hardware device.
	///
	/// \param device The device object to be used, which should have a unique name.
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(RawMultiAxisDevice* device);

	/// Unregisters the specified device object.
	/// The corresponding hardware device will become unused, and can be re-registered later.
	///
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const RawMultiAxisDevice* device);

	/// Sets the position scale for this device.
	void setPositionScale(const RawMultiAxisDevice* device, double scale);

	/// Sets the orientation scale for this device.
	void setOrientationScale(const RawMultiAxisDevice* device, double scale);

	/// Turns on or off the axis dominance setting for this device.
	void setAxisDominance(const RawMultiAxisDevice* device, bool onOff);

	/// Executes the operations for a single input frame for a single device.
	/// Should only be called from the context of the input loop thread.
	/// \param info The internal device data.
	/// \return true on success.
	bool runInputFrame(DeviceData* info);

	/// Executes the operations after the last input frame, as the device input loop thread is shutting down.
	/// Should only be called from the context of the input loop thread.
	/// \param info The internal device data.
	/// \return true on success.
	bool runAfterLastFrame(DeviceData* info);

	/// Updates the device information for a single device.
	/// \return true on success.
	bool updateDevice(DeviceData* info);

	/// Initializes the RawMultiAxis SDK.
	/// \return true on success.
	bool initializeSdk();

	/// Finalizes (de-initializes) the RawMultiAxis SDK.
	/// \return true on success.
	bool finalizeSdk();

	/// Scans hardware that is present in the system, and if an unused device is found, register an object for it.
	///
	/// \param device The device object to register if an unused device is found.
	/// \param [out] numUsedDevicesSeen The number of devices that were found during the scan but were already
	/// 	in use.  Can be used if the scan fails to determine the error message that should be displayed.
	/// \return true on success.
	bool findUnusedDeviceAndRegister(RawMultiAxisDevice* device, int* numUsedDevicesSeen);

	/// Register a device object given a device path, if the same path is not already in use.
	///
	/// \param path A unique system path that can be used to communicate with the device.
	/// \param device The device object to register if the index pair is in fact unused.
	/// \param [in,out] numUsedDevicesSeen The number of devices that were found during the scan but were
	/// 	already in use; incremented when an unused device is seen.
	/// \return true on success.
	bool registerIfUnused(const std::string& path, RawMultiAxisDevice* device, int* numUsedDevicesSeen);

	/// Creates the input loop thread.
	/// \return true on success.
	bool createPerDeviceThread(DeviceData* data);

	/// Destroys the input loop thread.
	/// \return true on success.
	bool destroyPerDeviceThread(DeviceData* data);

	/// Opens the specified device.
	/// \return The system-specific wrapper for the device.
	std::unique_ptr<SystemInputDeviceHandle> openDevice(const std::string& path);

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();



	/// Logger used by the scaffold and all devices.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_RAWMULTIAXISSCAFFOLD_H

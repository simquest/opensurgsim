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

#ifndef SURGSIM_DEVICES_TRACKIR_TRACKIRSCAFFOLD_H
#define SURGSIM_DEVICES_TRACKIR_TRACKIRSCAFFOLD_H

#include <memory>

#include "SurgSim/Framework/Logger.h"
#include "SurgSim/DataStructures/DataGroup.h"

namespace SurgSim
{
namespace Device
{

class TrackIRDevice;

/// A class that manages Natural Point TRACKIR devices.
///
/// \sa SurgSim::Device::TrackIRDevice
class TrackIRScaffold
{
public:
	/// Constructor.
	/// \param logger (optional) The logger to be used for the scaffold object and the devices it manages.
	/// 			  If unspecified or empty, a console logger will be created and used.
	explicit TrackIRScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger = nullptr);

	/// Destructor.
	~TrackIRScaffold();

	/// Gets the logger used by this object and the devices it manages.
	/// \return The logger used by this scaffold.
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const;

	/// Gets or creates the scaffold shared by all TrackIRDevice instances.
	/// The scaffold is managed using a SingleInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<TrackIRScaffold> getOrCreateSharedInstance();

	/// Sets the default log level.
	/// Has no effect unless called before a scaffold is created (i.e. before the first device).
	/// \param logLevel The log level.
	static void setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel);

//private:
	/// Internal shared state data type.
	struct StateData;
	/// Internal per-device information.
	struct DeviceData;
	/// Wrapper for the callback.
	class Callback;
	/// Wrapper for the handle.
	class Handle;

	friend class TrackIRDevice;
	friend class TrackIRThread;
	friend struct StateData;

	/// Registers the specified device object.
	/// If successful, the device object will become connected to an unused controller.
	///
	/// \param device The device object to be used, which should have a unique name.
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(TrackIRDevice* device);

	/// Unregisters the specified device object.
	/// The corresponding controller will become unused, and can be re-registered later.
	///
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const TrackIRDevice* device);

	/// Initializes a single device, creating the necessary HDAPI resources.
	/// \param [in,out] info	The device data.
	/// \return	true on success.
	bool initializeDeviceState(DeviceData* info);

	/// Finalizes a single device, destroying the necessary HDAPI resources.
	/// \param [in,out] info	The device data.
	/// \return	true on success.
	bool finalizeDeviceState(DeviceData* info);

	/// Updates the device information for a single device.
	/// \param info	The device data.
	/// \return	true on success.
	bool updateDevice(DeviceData* info);

	/// Initializes the OptiTrack SDK.
	/// \return true on success.
	bool initializeSdk();

	/// Finalizes (de-initializes) the OptiTrack SDK.
	/// \return true on success.
	bool finalizeSdk();

	/// Destroys the haptic loop callback.
	/// Should be called while NOT holding the internal device list mutex, to prevent deadlock.
	/// \return true on success.
	bool destroyHapticLoop();

	/// Starts the camera, it will start to sending frames.
	/// \return	true on success.
	bool startCamera();
	/// Stops the camera. 
	/// \return	true on success.
	bool stopCamera();

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

	/// Creates the input loop thread.
	/// \return true on success.
	bool createPerDeviceThread(DeviceData* data);
	/// Destroys the input loop thread.
	/// \return true on success.
	bool destroyPerDeviceThread(DeviceData* data);

	/// Check for OptiTrack HDAPI errors, display them, and signal fatal errors.
	/// \param message An additional descriptive message.
	/// \return true if there was a fatal error; false if everything is OK.
	//bool checkForFatalError(const char* message);

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();

	/// Logger used by the scaffold and all devices.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;

	/// The default logging level.
	static SurgSim::Framework::LogLevel m_defaultLogLevel;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_TRACKIR_TRACKIRSCAFFOLD_H

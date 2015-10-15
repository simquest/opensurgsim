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

namespace SurgSim
{

namespace DataStructures
{
class DataGroup;
}

namespace Devices
{
class TrackIRDevice;

/// A class that manages Natural Point TRACKIR devices.
///
/// \sa SurgSim::Devices::TrackIRDevice
class TrackIRScaffold
{
public:
	/// Constructor.
	TrackIRScaffold();

	/// Destructor.
	~TrackIRScaffold();

	/// Gets or creates the scaffold shared by all TrackIRDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<TrackIRScaffold> getOrCreateSharedInstance();

private:
	/// Internal shared state data type.
	struct StateData;
	/// Internal per-device information.
	struct DeviceData;

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

	/// Sets the position scale for the device.
	/// \param device The device whose position scale will be set.
	/// \param scale Scale of the position.
	void setPositionScale(const TrackIRDevice* device, double scale);
	/// Sets the orientation scale for the device.
	/// \param device The device whose orientation scale will be set.
	/// \param scale Scale of the orientation.
	void setOrientationScale(const TrackIRDevice* device, double scale);

	/// Initializes the OptiTrack SDK.
	/// \return true on success.
	bool initializeSdk();
	/// Finalizes (de-initializes) the OptiTrack SDK.
	/// \return true on success.
	bool finalizeSdk();

	/// Start the camera, it will start sending frames.
	/// \param info DeviceData object which contains the camera to be started.
	/// \return	true on success.
	bool startCamera(DeviceData* info);
	/// Stop the camera, it will stop sending frames.
	/// \param info DeviceData object which contains the camera to be stopped.
	/// \return	true on success.
	bool stopCamera(DeviceData* info);

	/// Executes the operations for a single input frame for a single device.
	/// Should only be called from the context of the input loop thread.
	/// \param info The internal device data.
	/// \return true on success.
	bool runInputFrame(DeviceData* info);

	/// Updates the device information for a single device.
	/// \param info	The device data.
	/// \return	true on success.
	bool updateDevice(DeviceData* info);

	/// Creates a thread for the given DeviceData object
	/// \param data A DeviceData object
	/// \return true on success.
	bool createPerDeviceThread(DeviceData* data);
	/// Destroys the thread associated with the given DeviceData object
	/// \param data A DeviceData object
	/// \return true on success.
	bool destroyPerDeviceThread(DeviceData* data);

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();

	/// Logger used by the scaffold and all devices.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_TRACKIR_TRACKIRSCAFFOLD_H

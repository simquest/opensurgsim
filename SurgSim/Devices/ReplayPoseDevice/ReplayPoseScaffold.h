// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DEVICES_REPLAYPOSEDEVICE_REPLAYPOSESCAFFOLD_H
#define SURGSIM_DEVICES_REPLAYPOSEDEVICE_REPLAYPOSESCAFFOLD_H

#include <memory>

#include <boost/thread/mutex.hpp>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/BasicThread.h"
#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim
{
namespace Devices
{

class ReplayPoseDevice;

class ReplayPoseScaffold : public SurgSim::Framework::BasicThread
{
	friend class ReplayPoseDevice;

public:
	/// Constructor.
	ReplayPoseScaffold();

	/// Destructor.
	~ReplayPoseScaffold();

	/// Gets or creates the scaffold shared by all RawMultiAxisDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<ReplayPoseScaffold> getOrCreateSharedInstance();

protected:
	bool doInitialize() override;
	bool doStartUp() override;
	bool doUpdate(double dt) override;

private:
	/// Internal per-device information.
	struct DeviceData;

	/// Registers the specified device object.
	/// If successful, the device object will become connected to an hardware device.
	/// \param device The device object to be used, which should have a unique name.
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(ReplayPoseDevice* device);

	/// Unregisters the specified device object.
	/// The corresponding controller will become unused, and can be re-registered later.
	/// \return True on success, false on failure.
	bool unregisterDevice();

	/// Updates the device information for a single device.
	/// \param info The information to update the device from
	/// \return	True on success.
	bool updateDevice(ReplayPoseScaffold::DeviceData* info);

	/// Builds the data layout for the application input (i.e. device output).
	static DataStructures::DataGroup buildDeviceInputData();

	/// Logger used by the scaffold and all devices.
	std::shared_ptr<Framework::Logger> m_logger;

	/// The ReplayPose device locking mechanism
	boost::mutex m_deviceLock;

	/// The ReplayPose device managed by this scaffold
	std::unique_ptr<DeviceData> m_device;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_REPLAYPOSEDEVICE_REPLAYPOSESCAFFOLD_H

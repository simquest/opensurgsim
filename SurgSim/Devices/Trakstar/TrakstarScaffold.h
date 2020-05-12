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

#ifndef SURGSIM_DEVICES_TRAKSTAR_TRAKSTARSCAFFOLD_H
#define SURGSIM_DEVICES_TRAKSTAR_TRAKSTARSCAFFOLD_H

#include <memory>

#include "SurgSim/Framework/BasicThread.h"

namespace SurgSim
{

namespace DataStructures
{
class DataGroup;
}
namespace Framework
{
class Barrier;
}

namespace Devices
{
class TrakstarDevice;

/// A class that manages Natural Point TRAKSTAR devices.
///
/// \sa SurgSim::Devices::TrakstarDevice
class TrakstarScaffold : public SurgSim::Framework::BasicThread
{
public:
	/// Constructor.
	TrakstarScaffold();

	/// Destructor.
	~TrakstarScaffold();

	/// Gets or creates the scaffold shared by all TrakstarDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<TrakstarScaffold> getOrCreateSharedInstance();

private:
	/// Internal shared state data type.
	struct StateData;
	/// Internal per-device information.
	struct DeviceData;

	friend class TrakstarDevice;
	friend class TrakstarThread;
	friend struct StateData;

	/// Checks if return code is an error.  Logs if it is.
	/// \param error The error code from the ATC 3DG SDK.
	/// \return true if the error code represents "success", false if there is a problem.
	bool isSuccess(int error);

	/// Initializes the ATC 3DG SDK.
	/// \return true on success.
	bool initializeSdk();
	/// Finalizes (de-initializes) the ATC 3DG SDK.
	/// \return true on success.
	bool finalizeSdk();

	/// Registers the specified device object.
	/// If successful, the device object will become connected to an unused sensor.
	///
	/// \param device The device object to be used, which should have a unique name.
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(TrakstarDevice* device);
	/// Unregisters the specified device object.
	/// The corresponding sensor will become unused, and can be re-registered later.
	///
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const TrakstarDevice* device);

	/// Initialize this thread.
	/// \return True on success, false otherwise.
	bool doInitialize() override;
	/// Start up this thread.
	/// \return True on success, false otherwise.
	bool doStartUp() override;
	/// Update work of this thread.
	/// \param dt The time step.
	/// \return True on success, false otherwise.
	bool doUpdate(double dt) override;
	/// Update the input data for one device.
	/// \param info The device data.
	void updateDevice(DeviceData* info);
	
	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();

	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;

	/// Barrier used to make sure the thread/SDK initializes before SDK calls are made.
	std::shared_ptr<Framework::Barrier> m_barrier;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_TRAKSTAR_TRAKSTARSCAFFOLD_H

// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DEVICE_OCULUSSCAFFOLD_H
#define SURGSIM_DEVICE_OCULUSSCAFFOLD_H

#include <memory>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/BasicThread.h"

namespace SurgSim
{
namespace Framework
{
class Logger;
}
};

namespace SurgSim
{

namespace Device
{
class OculusDevice;

/// A class that manages Oculus Rift DK2 devices.
///
/// \sa SurgSim::Device::OculusDevice
class OculusScaffold : SurgSim::Framework::BasicThread
{
public:
	/// Destructor.
	~OculusScaffold();

protected:
	bool doInitialize() override;
	bool doStartUp() override;
	bool doUpdate(double dt) override;

private:
	/// Internal shared state data type.
	struct StateData;

	/// Internal per-device information.
	struct DeviceData;

	friend class OculusDevice;

	/// Constructor.
	OculusScaffold();

	/// Gets or creates the scaffold shared by all OculusDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<OculusScaffold> getOrCreateSharedInstance();

	/// Registers the specified device object.
	/// \param device The device object to be used, which should have a unique name.
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(OculusDevice* device);

	/// Unregisters the specified device object.
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const OculusDevice* device);

	/// Initializes Oculus SDK.
	/// \return true on success; false otherwise.
	bool initializeSdk();

	/// Finalizes (de-initializes) Oculus SDK.
	/// \return true on success; false otherwise.
	bool finalizeSdk();

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();

	/// Logger used by the scaffold and all devices.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;
};

};  // namespace Device
};  // namespace SurgSim 

#endif  // SURGSIM_DEVICE_OCULUSSCAFFOLD_H

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

#ifndef SURGSIM_DEVICES_OPENNI_OPENNISCAFFOLD_H
#define SURGSIM_DEVICES_OPENNI_OPENNISCAFFOLD_H

#include <memory>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/BasicThread.h"

namespace SurgSim
{

namespace Framework
{
class Logger;
};

namespace Device
{

class OpenNIDevice;

/// A class that manages OpenNI devices
///
/// \sa SurgSim::Device::OpenNIDevice
class OpenNIScaffold : public SurgSim::Framework::BasicThread
{
public:
	/// Destructor.
	virtual ~OpenNIScaffold();

protected:
	bool doInitialize() override;

	bool doStartUp() override;

	bool doUpdate(double dt) override;

private:
	/// Internal shared state data type.
	struct StateData;

	/// Interal per-device information.
	struct DeviceData;

	friend class OpenNIDevice;

	/// Constructor.
	OpenNIScaffold();

	/// Gets or creates the scaffold shared by all OpenNIDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<OpenNIScaffold> getOrCreateSharedInstance();

	/// Registers the specified device object.
	/// \param device The device object to be used
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(OpenNIDevice* device);

	/// Unregisters the specified device object.
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const OpenNIDevice* device);

	/// Do the OpenNI specific registration
	/// \param info The device data
	/// \return true on success, false on failure.
	bool doRegisterDevice(DeviceData* info);

	/// Do the OpenNI specific unregistration
	/// \param info The device data
	/// \return true on success, false on failure.
	bool doUnregisterDevice(DeviceData* info);

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();

	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;

	/// Logger used by the scaffold
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
};

};  // namespace Device
};  // namespace SurgSim

#endif // SURGSIM_DEVICES_OPENNI_OPENNISCAFFOLD_H

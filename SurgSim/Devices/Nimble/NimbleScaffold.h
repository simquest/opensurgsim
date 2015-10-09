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

#ifndef SURGSIM_DEVICES_NIMBLE_NIMBLESCAFFOLD_H
#define SURGSIM_DEVICES_NIMBLE_NIMBLESCAFFOLD_H

#include <memory>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/BasicThread.h"
#include "SurgSim/Framework/Logger.h"

namespace SurgSim
{
namespace Devices
{

class NimbleDevice;
class NimbleThread;

/// A class that manages Nimble devices.
///
/// \sa SurgSim::Devices::NimbleDevice
class NimbleScaffold : public SurgSim::Framework::BasicThread
{
public:
	/// Destructor.
	~NimbleScaffold();

	/// Gets or creates the scaffold shared by all NimbleDevice instances.
	/// The scaffold is managed using a SingleInstance object.
	/// \return the scaffold object.
	static std::shared_ptr<NimbleScaffold> getOrCreateSharedInstance();

protected:
	bool doInitialize() override;
	bool doStartUp() override;
	bool doUpdate(double dt) override;
	void doBeforeStop() override;

private:
	/// Internal shared state data type.
	struct StateData;

	friend class NimbleDevice;
	friend class NimbleThread;

	/// Constructor.
	NimbleScaffold();

	/// Registers the specified device object.
	/// \param device The device object to be used.
	/// \return True if the initialization succeeds, false if it fails.
	/// \note There can be only one NimbleDevice at a time.
	bool registerDevice(NimbleDevice* device);

	/// Unregisters the specified device object.
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const NimbleDevice* device);

	/// Update the devices based on the data read from the Nimble server.
	void updateDeviceData();

	/// Reset the device data.
	void resetDeviceData();

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();

	/// Logger used by the scaffold and all devices.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;

	/// The IP address of the Nimble hand tracking server.
	std::string m_serverIpAddress;
	/// The port where the server is communicating.
	std::string m_serverPort;
	/// Flag to indicate that the socket is opened successfully.
	bool m_serverSocketOpen;

	/// The data group name for the joint poses, and the corresponding indices within the state data.
	static std::array<std::pair<std::string, int>, 15> m_jointPoseNames;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_NIMBLE_NIMBLESCAFFOLD_H
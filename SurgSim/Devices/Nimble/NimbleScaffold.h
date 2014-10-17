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
#include "SurgSim/Framework/Logger.h"

namespace SurgSim
{
namespace Device
{

class NimbleDevice;
class NimbleThread;

/// A class that manages Nimble devices.
///
/// \sa SurgSim::Device::NimbleDevice
class NimbleScaffold
{
public:
	/// Destructor.
	~NimbleScaffold();

	/// Gets the logger used by this object and the devices it manages.
	/// \return The logger.
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const;

	/// Gets or creates the scaffold shared by all NimbleDevice instances.
	/// The scaffold is managed using a SingleInstance object.
	/// \return the scaffold object.
	static std::shared_ptr<NimbleScaffold> getOrCreateSharedInstance();

private:
	/// Internal shared state data type.
	struct StateData;

	friend class NimbleDevice;
	friend class NimbleThread;

	/// Constructor.
	/// \param logger (optional) The logger to be used for the scaffold object and the devices it manages.
	/// 			  If unspecified or empty, a console logger will be created and used.
	explicit NimbleScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger = nullptr);

	/// Registers the specified device object.
	/// \param device The device object to be used.
	/// \return True if the initialization succeeds, false if it fails.
	/// \note There can be only one NimbleDevice at a time.
	bool registerDevice(NimbleDevice* device);

	/// Unregisters the specified device object.
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const NimbleDevice* device);

	/// Initialize the state and establish connection with the Nimble server.
	/// \returns True, if the connection is Nimble server is established.
	bool initialize();

	/// Listens to the Nimble server and de-serializes the data that is read.
	/// \return True, if everything went well and the thread can continue to run.
	bool update();

	/// Close communication with the Nimble server.
	void finalize();

	/// Update the devices based on the data read from the Nimble server.
	void updateDeviceData();

	/// Reset the device data.
	void resetDeviceData();

	/// Creates the hand tracking client thread.
	/// \return true on success.
	bool createThread();

	/// Destroys the hand tracking client thread.
	/// Should be called while NOT holding the internal device list mutex, to prevent deadlock.
	/// \return true on success.
	bool destroyThread();

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

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_NIMBLE_NIMBLESCAFFOLD_H
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

#ifndef SURGSIM_DEVICES_LABJACK_LABJACKSCAFFOLD_H
#define SURGSIM_DEVICES_LABJACK_LABJACKSCAFFOLD_H

#include <memory>

namespace SurgSim
{
namespace DataStructures
{
class DataGroup;
};

namespace Framework
{
class Logger;
}

namespace Device
{

class LabJackDevice;
class LabJackThread;

/// A class that implements the behavior of LabJackDevice objects.
/// \sa SurgSim::Device::LabJackDevice
class LabJackScaffold
{
public:
	/// Internal per-device information.  This is public because it is passed to the LabJackThread and back.
	struct DeviceData;

	/// Constructor.
	LabJackScaffold();

	/// Destructor.
	~LabJackScaffold();

	/// Gets the logger used by this object and the devices it manages.
	/// \return The logger.
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const;

	/// Gets or creates the scaffold shared by all LabJackDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<LabJackScaffold> getOrCreateSharedInstance();

	/// Does one-time configuration of the LabJack for timers, counters, and analog inputs.
	/// Must be called by the LabJackThread because the LabJack separates all commands by the calling thread.
	/// \param device The internal device data.
	void configureDevice(DeviceData* device);

private:
	/// Internal shared state data type.
	struct StateData;

	/// Wrapper for the LabJack device handle.
	class Handle;

	friend class LabJackDevice;
	friend class LabJackThread;
	friend struct StateData;

	/// Registers the specified device object.
	/// If successful, the device object will become connected to an unused hardware device.
	/// \param device The device object to be used, which should have a unique name.
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(LabJackDevice* device);

	/// Unregisters the specified device object.
	/// The corresponding hardware device will become unused, and can be re-registered later.
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const LabJackDevice* device);

	/// Executes the operations for a single input frame for a single device.
	/// Should only be called from the context of the input loop thread.
	/// \param info The internal device data.
	/// \return True to keep the LabJackThread running.
	bool runInputFrame(DeviceData* info);

	/// Updates the device information for a single device.
	/// \param info The internal device data.
	/// \return true on success.
	bool updateDevice(DeviceData* info);

	/// Destroys the input loop thread.
	/// \param data The internal device data.
	/// \return true on success.
	bool destroyPerDeviceThread(DeviceData* data);

	/// One-time configuration of the clock and timers.
	/// \param deviceData The internal device data.
	/// \return False if any errors.
	bool configureClockAndTimers(DeviceData* deviceData);

	/// One-time configuration of the number of timers.
	/// \param deviceData The internal device data.
	/// \return False if any errors.
	bool configureNumberOfTimers(DeviceData* deviceData);

	/// One-time configuration of the clock.
	/// \param deviceData The internal device data.
	/// \return False if any errors.
	bool configureClock(DeviceData* deviceData);

	/// One-time configuration of the timers.
	/// \param deviceData The internal device data.
	/// \return False if any errors.
	bool configureTimers(DeviceData* deviceData);

	/// One-time configuration of the digital inputs and outputs.
	/// \param deviceData The internal device data.
	/// \return False if any errors.
	bool configureDigital(DeviceData* deviceData);

	/// One-time configuration of the analog inputs.
	/// \param deviceData The internal device data.
	/// \return False if any errors.
	bool configureAnalog(DeviceData* deviceData);

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();

	/// Logger used by the scaffold and all devices.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_LABJACK_LABJACKSCAFFOLD_H

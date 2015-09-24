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

#ifndef SURGSIM_DEVICES_PHANTOM_PHANTOMSCAFFOLD_H
#define SURGSIM_DEVICES_PHANTOM_PHANTOMSCAFFOLD_H

#include <memory>

#include "SurgSim/Framework/Logger.h"
#include "SurgSim/DataStructures/DataGroup.h"

namespace SurgSim
{
namespace Devices
{

class PhantomDevice;

/// A class that manages Sensable PHANTOM devices.
///
/// This should support any PHANTOM device that can communicate using OpenHaptics 3.0 toolkit, such as PHANTOM
/// Omni, PHANTOM Desktop, and the PHANTOM Premium series devices.  The implementation is currently limited to
/// 3DoF haptic output (forces only, no torques).
///
/// \sa SurgSim::Devices::PhantomDevice
class PhantomScaffold
{
public:
	/// Constructor.
	PhantomScaffold();

	/// Destructor.
	~PhantomScaffold();

	/// Gets or creates the scaffold shared by all PhantomDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<PhantomScaffold> getOrCreateSharedInstance();

private:
	/// Internal shared state data type.
	struct StateData;
	/// Interal per-device information.
	struct DeviceData;
	/// Wrapper for the haptic loop callback.
	class Callback;
	/// Wrapper for the OpenHaptics device handle.
	class Handle;

	friend class PhantomDevice;

	/// Registers the specified device object.
	/// If successful, the device object will become connected to an unused controller.
	///
	/// \param device The device object to be used, which should have a unique name.
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(PhantomDevice* device);

	/// Unregisters the specified device object.
	/// The corresponding controller will become unused, and can be re-registered later.
	///
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const PhantomDevice* device);

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

	/// Calculates forces and torques and sends them to the device library.  The force to output is
	/// composed of a vector named "force" in the output data, plus contributions from two optional Jacobians.
	/// If the matrix "springJacobian" is provided in the output data, a spring force & torque are generated from
	/// its product with the difference between the current pose and the pose in the output data named "inputPose".
	/// A damping force & torque are generated similarly.  The intention is for a Behavior to calculate a nominal
	/// force & torque as well as the desired linearized changes to the force & torque based on changes to the input's
	/// pose & velocities.  Then the linearized deltas to the output force & torque can be calculated at the haptic
	/// update rates, thereby smoothing the output response to motion.
	/// \param info The device data.
	void calculateForceAndTorque(DeviceData* info);

	/// Sets the input DataGroup, which will be pushed to the InputComponent
	/// \param info The device data
	void setInputData(DeviceData* info);

	/// Executes the operations for a single haptic frame.
	/// Should only be called from the context of an OpenHaptics callback.
	/// \return true on success.
	bool runHapticFrame();

	/// Creates the haptic loop callback.
	/// \return true on success.
	bool createHapticLoop();

	/// Destroys the haptic loop callback.
	/// Should be called while NOT holding the internal device list mutex, to prevent deadlock.
	/// \return true on success.
	bool destroyHapticLoop();

	/// Starts the OpenHaptics scheduler.
	/// \return	true on success.
	bool startScheduler();

	/// Stops the OpenHaptics scheduler.
	/// \return	true on success.
	bool stopScheduler();

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();

	/// Logger used by the scaffold and all devices.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_PHANTOM_PHANTOMSCAFFOLD_H

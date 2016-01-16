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

#ifndef SURGSIM_DEVICES_NOVINT_NOVINTSCAFFOLD_H
#define SURGSIM_DEVICES_NOVINT_NOVINTSCAFFOLD_H

#include <memory>
#include <string>

#include "SurgSim/DataStructures/DataGroup.h"

namespace SurgSim
{
namespace Devices
{

class NovintDevice;

/// A class that manages Novint Falcon devices.
///
/// This should support any device that can communicate using the Novint HDAL SDK toolkit, such as the
/// off-the-shelf Novint Falcon gaming controller and the Novint Falcon with the Open Surgery Grip.
///
/// \sa SurgSim::Devices::NovintDevice
class NovintScaffold
{
public:
	/// Constructor.
	NovintScaffold();

	/// Destructor.
	~NovintScaffold();

	/// Gets or creates the scaffold shared by all NovintDevice and Novint7DofDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<NovintScaffold> getOrCreateSharedInstance();

	void setAuxiliary(int grip, double roll, double toolDof);

private:
	NovintScaffold(const NovintScaffold&) /*= delete*/;
	NovintScaffold& operator=(const NovintScaffold&) /*= delete*/;

	/// Internal shared state data type.
	struct StateData;
	/// Internal per-device information.
	struct DeviceData;
	/// Wrapper for the haptic loop callback.
	class Callback;
	/// Wrapper for the HDAL device handle.
	class Handle;

	friend class NovintDevice;

	/// Registers the specified device object, and starts the servo loop if necessary.
	/// If successful, the device object will become connected to an unused controller.
	///
	/// \param device The device object to be used, which should have a unique name.
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(NovintDevice* device);

	/// Unregisters the specified device object.
	/// The corresponding controller will become unused, and can be re-registered later.
	///
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const NovintDevice* device);

	/// Initializes a single device, creating the necessary HDAL resources.
	/// \param [in,out] info	The device data.
	/// \return	true on success.
	bool initializeDeviceState(DeviceData* info);

	/// Get the Handle associated with an initialization name, if any.
	/// \param initializationName The initialization name (from the configuration file).
	/// \return Shared pointer to Handle, or nullptr if not found.
	std::shared_ptr<NovintScaffold::Handle> findHandleByInitializationName(const std::string& initializationName);

	/// Updates the device information for a single device's input.
	/// \param info	The device data.
	/// \return	true on success.
	bool updateDeviceInput(DeviceData* info);

	/// Updates the device information for a single device's output.  If pullOutput failed, zeros forces & torques.
	/// \param info	The device data.
	/// \param pulledOutput true if the most recent pullOutput succeeded.
	/// \return	true on success.
	bool updateDeviceOutput(DeviceData* info, bool pulledOutput);

	/// Checks whether a device has been homed.  If the position and/or orientation have not been homed, zeros the
	/// respective Values.  Call this before setting the data to send to the Input Component.  The DeviceData's
	/// parameter mutex should be locked before this function is called.
	/// \param info The device data.
	void checkDeviceHoming(DeviceData* info);

	/// Calculates forces, and torques if the device is a 7Dof, and sends them to the HDAL.  The force to output is
	/// composed of a vector named "force" in the output data, plus contributions from two optional Jacobians.
	/// If the matrix "springJacobian" is provided in the output data, a spring force & torque are generated from
	/// its product with the difference between the current pose and the pose in the output data named "inputPose".
	/// A damping force & torque are generated similarly.  The intention is for a Behavior to calculate a nominal
	/// force & torque as well as the desired linearized changes to the force & torque based on changes to the input's
	/// pose & velocities.  Then the linearized deltas to the output force & torque can be calculated at the haptic
	/// update rates, thereby smoothing the output response to motion.
	/// \param info The device data.
	/// \note The DeviceData's parameter mutex should be locked before this function is called.
	void calculateForceAndTorque(DeviceData* info);

	/// Sets the input DataGroup, which will be pushed to the InputComponent
	/// \param info The device data
	void setInputData(DeviceData* info);

	/// Gets the map from name to serial number.
	/// \return The map.
	std::map<std::string, std::string> getNameMap();

	/// Creates a NovintScaffold::Handle for each device connected when the first registerDevice is called.
	/// \return True if there are device handles created; false otherwise.
	bool createAllHandles();

	/// Destroys all the initialized handles.
	void destroyAllHandles();

	/// Store the handle if it is valid.
	/// \return true If handle is valid and stored; false, otherwise.
	bool storeHandleIfValid(const std::shared_ptr<Handle>& handle, const std::string& serial);

	/// Executes the operations for a single haptic frame.
	/// Should only be called from the context of a HDAL callback.
	void runHapticFrame();

	/// Gets the gravity compensation flag for the current device.
	/// \param info	The device data.
	/// \param [out] gravityCompensationState	State of the gravity compensation flag.
	/// \return	true if it succeeds, false if it fails.
	bool getGravityCompensation(const DeviceData* info, bool* gravityCompensationState);

	/// Attempts to force the gravity compensation flag for the current device to the specified value.
	///
	/// Sets the flag repeatedly, until it reports the desired value, in order to work around the problem where
	/// attempts to set the gravity compensation do not always change the actual gravity compensation flag in the
	/// device.
	///
	/// Logs a message if the state is known to have been changed.
	///
	/// \param info	The device data.
	/// \param gravityCompensationState	Desired state of the gravity compensation flag.
	/// \return	true if it succeeds, false if it fails.
	bool enforceGravityCompensation(const DeviceData* info, bool gravityCompensationState);

	/// Sets the gravity compensation flag for the current device, unless it's already set as desired.
	/// \param info	The device data.
	/// \param gravityCompensationState	Desired state of the gravity compensation flag.
	/// \return	true if it succeeds, false if it fails.
	bool setGravityCompensation(const DeviceData* info, bool gravityCompensationState);

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();

	/// Sets the position scale for this device.
	/// \param device A pointer to the device.
	/// \param scale The multiplicative factor to apply to the position.
	void setPositionScale(const NovintDevice* device, double scale);

	/// Sets the orientation scale for this device.
	/// \param device A pointer to the device.
	/// \param scale The multiplicative factor to apply to the rotation angles.
	void setOrientationScale(const NovintDevice* device, double scale);

	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_NOVINT_NOVINTSCAFFOLD_H

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

#ifndef SURGSIM_DEVICES_NOVINT_NOVINTDEVICE_H
#define SURGSIM_DEVICES_NOVINT_NOVINTDEVICE_H

#include <string>

#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Devices
{
class NovintScaffold;

SURGSIM_STATIC_REGISTRATION(NovintDevice);

/// A class implementing the communication with a Novint Falcon device.
///
/// This should provide basic support for any device that can communicate using the Novint HDAL SDK toolkit, such as
/// the off-the-shelf Novint Falcon haptic gaming controller.  Note that certain devices may require device-specific
/// support in the code to enable particular hardware features.  In particular, the Novint Falcon with the Open Surgery
/// Grip will not be able to produce torques unless it is accessed using the Novint7DofHapticDevice class.
///
/// \par Application input provided by the device:
///   | type       | name                  |                                                      |
///   | ----       | ----                  | ---                                                  |
///   | pose       | "pose"                | %Device pose (units are meters).                     |
///   | scalar     | "toolDof"             | %7th Dof (e.g., handle open/close angle)             |
///   | bool       | "button1"             | %State of the first device button if present.        |
///   | bool       | "button2"             | %State of the second device button if present.       |
///   | bool       | "button3"             | %State of the third device button if present.        |
///   | bool       | "button4"             | %State of the third device button if present.        |
///   | bool       | "isHomed"             | %Device homing status.                               |
///   | bool       | "isPositionHomed"     | %Device homing status, position only.                |
///   | bool       | "isOrientationHomed"  | %Device homing status, orientation only.             |
/// Note that \c button1 through \c 4 correspond to the buttons 0 through 3 provided by the
/// HDAL SDK, but a custom Novint device might have fewer than 4 buttons.
///
/// \par Application output used by the device:
///   | type       | name                  |                                                      |
///   | ----       | ----                  | ---                                                  |
///   | vector     | "force"               | %Device output force (units are newtons).            |
///   | vector     | "torque"              | %Device output torque (in newton-meters, 7Dof only). |
///   | bool       | "gravityCompensation" | Enable or disable hardware gravity compensation.     |
///
class NovintDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	/// \param name A unique name for the device that will be used by the application.
	explicit NovintDevice(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Devices::NovintDevice);

	/// Destructor.
	virtual ~NovintDevice();

	/// Sets the serial number used to register this device with the hardware library.
	/// An empty string indicates the first available device.
	/// \param serialNumber The serial number.
	/// \sa setInitializationName
	void setSerialNumber(const std::string& serialNumber);

	/// Gets the serial number used to register this device with the hardware library.
	/// \param [out] serialNumber The serial number, if set.
	/// \return	true if the serialNumber has been set.
	bool getSerialNumber(std::string* serialNumber) const;

	/// Sets the name used to register this device with the hardware library.
	/// To use an initialization name, the 'devices.yaml' file must be in the working directory,
	/// (see example at Data/devices.yaml) with an entry of the form "initializationName: serialNumber".
	/// An empty string indicates the first available device.
	/// \param initializationName The initialization name.
	/// \sa setSerialNumber
	void setInitializationName(const std::string& initializationName);

	/// Gets the name used to register this device with the hardware library.
	/// \param [out] initializationName The initialization name, if set.
	/// \return	true if the initializationName has been set
	bool getInitializationName(std::string* initializationName) const;

	bool initialize() override;

	bool isInitialized() const override;

	/// Sets the position scale for this device.
	/// The position scale controls how much the pose changes for a given device translation.
	/// The default value for a raw device tries to correspond to the actual physical motion of the device.
	/// \param scale The multiplicative factor to apply to the position.
	void setPositionScale(double scale);
	/// Gets the position scale for this device.
	double getPositionScale() const;

	/// Sets the orientation scale for this device.
	/// The orientation scale controls how much the pose changes for a given device rotation.
	/// The default value for a raw device tries to correspond to the actual physical motion of the device.
	/// \param scale The multiplicative factor to apply to the rotation angles.
	void setOrientationScale(double scale);
	/// Gets the orientation scale for this device.
	double getOrientationScale() const;

	/// Sets whether or not this is supposed to be a 7Dof device.
	/// If the hardware is not actually 7Dof, the resulting device will never home.
	/// If the hardware is 7Dof, and this is not called, the resulting device will act like a typical 3Dof.
	/// \param val true for a 7Dof device.
	void set7DofDevice(bool val);
	/// Query if this object represents a 7 degree of freedom hardware device.
	/// \return	true if 7 degree of freedom device, false if not.
	virtual bool is7DofDevice() const;

	/// Set the maximum force that can be sent to the device. Higher force values will be scaled to this magnitude.
	/// Generally Falcons are robust enough that commanding excessive forces will not cause a problem,
	/// but in-development systems may overheat if over-driven.
	/// \param force The maximum force magnitude (in Newtons) that will be sent to the hardware.
	void setMaxForce(double force);

	/// \return The maximum force (in Newtons) that can be sent to the device.
	double getMaxForce() const;

private:
	friend class NovintScaffold;

	bool finalize() override;

	///@{
	/// Used for serializing optional properties
	const DataStructures::OptionalValue<std::string>& getOptionalInitializationName() const;
	void setOptionalInitializationName(const DataStructures::OptionalValue<std::string>& name);
	const DataStructures::OptionalValue<std::string>& getOptionalSerialNumber() const;
	void setOptionalSerialNumber(const DataStructures::OptionalValue<std::string>& serial);
	///@}

	/// The scaffold handles all the communication with the SDK.
	std::shared_ptr<NovintScaffold> m_scaffold;

	/// The name passed to the SDK to specify which hardware device should be used.
	SurgSim::DataStructures::OptionalValue<std::string> m_initializationName;

	/// The serial number passed to the SDK to specify which hardware device should be used.
	SurgSim::DataStructures::OptionalValue<std::string> m_serialNumber;

	/// Scale factor for the position axes; stored locally before the device is initialized.
	double m_positionScale;
	/// Scale factor for the orientation axes; stored locally before the device is initialized.
	double m_orientationScale;

	/// True if the device is 7Dof, false if the device is 3Dof.
	bool m_7DofDevice;

	/// The maximum force magnitude (in Newtons) that should be sent to the hardware.
	double m_maxForce;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_NOVINT_NOVINTDEVICE_H

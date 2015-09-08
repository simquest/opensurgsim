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

#ifndef SURGSIM_DEVICES_PHANTOM_PHANTOMDEVICE_H
#define SURGSIM_DEVICES_PHANTOM_PHANTOMDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Device
{
class PhantomScaffold;

SURGSIM_STATIC_REGISTRATION(PhantomDevice);

/// A class implementing the communication with a SensAble/Geomagic PHANTOM device.
///
/// This should support any device that can communicate using the OpenHaptics 3.x toolkit, such as the
/// PHANTOM Omni (a.k.a. Geomagic Touch), PHANTOM Desktop (a.k.a. Geomagic Touch X), and the PHANTOM Premium
/// series devices.
///
/// \par Application input provided by the device:
///   | type       | name        |                                                                |
///   | ----       | ----        | ---                                                            |
///   | pose       | "pose"      | %Device pose (units are meters).                               |
///   | bool       | "button1"   | %State of the first device button.                             |
///   | bool       | "button2"   | %State of the second device button if present.                 |
///   | bool       | "button3"   | %State of the third device button (probably doesn't exist).    |
///   | bool       | "button4"   | %State of the third device button (probably doesn't exist).    |
/// Note that \c button1 through \c 4 correspond to the \c HD_DEVICE_BUTTON_1 through \c 4 provided by the
/// OpenHaptics API, but your PHANTOM device likely has fewer than 4 buttons.  On one-button PHANTOM devices,
/// the button state can be accessed through \c button1.  On a PHANTOM Omni or a Geomagic Touch, \c button1
/// corresponds to the front (blue) stylus button, and \c button2 to the rear (white/gray) stylus button.
///
/// \par Application output used by the device:
///   | type       | name        |                                                                |
///   | ----       | ----        | ---                                                            |
///   | vector     | "force"     | %Device output force (units are newtons).                      |
///   | vector     | "torque"    | %Device output torque (units are Nm).                          |
///
/// \sa SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class PhantomDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	explicit PhantomDevice(const std::string& uniqueName);

	SURGSIM_CLASSNAME(SurgSim::Device::PhantomDevice);

	/// Destructor.
	virtual ~PhantomDevice();

	/// Sets the name used to register this device with the hardware library.
	/// \param initializationName The name passed to HDAPI when initializing the device.  This should match a
	/// 	configured PHANTOM device; alternately, an empty string indicates the default device.
	void setInitializationName(const std::string& initializationName);

	/// Gets the name used by the Phantom device configuration to refer to this device.
	/// Note that this may or may not be the same as the device name retrieved by getName().
	/// An empty string indicates the default device.
	/// \return	The initialization name.
	std::string getInitializationName() const;

	bool initialize() override;

	bool finalize() override;

	/// Check whether this device is initialized.
	bool isInitialized() const;

private:
	friend class PhantomScaffold;

	std::shared_ptr<PhantomScaffold> m_scaffold;
	std::string m_initializationName;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_PHANTOM_PHANTOMDEVICE_H

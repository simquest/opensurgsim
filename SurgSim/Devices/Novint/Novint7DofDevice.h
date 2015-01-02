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

#ifndef SURGSIM_DEVICES_NOVINT_NOVINT7DOFDEVICE_H
#define SURGSIM_DEVICES_NOVINT_NOVINT7DOFDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Devices/Novint/NovintCommonDevice.h"

namespace SurgSim
{
namespace Device
{


/// A class implementing the communication with a Novint Falcon with the Open Surgery Grip 7-DoF device.
///
/// \par Application input provided by the device:
///   | type       | name        |                                                                |
///   | ----       | ----        | ---                                                            |
///   | pose       | "pose"      | %Device pose (units are meters).                               |
///   | bool       | "button1"   | %Always false (there are no buttons present).                  |
///   | bool       | "button2"   | %Always false (there are no buttons present).                  |
///   | bool       | "button3"   | %Always false (there are no buttons present).                  |
///   | bool       | "button4"   | %Always false (there are no buttons present).                  |
///   | bool       | "isHomed"   | %Device homing status.                                         |
///   | bool       | "isHeld"    | %Device homing status.                                         |
///
/// \par Application output used by the device:
///   | type       | name                  |                                                      |
///   | ----       | ----                  | ---                                                  |
///   | vector     | "force"               | %Device output force (units are newtons).            |
///   | vector     | "torque"              | %Device output torque (units are newton-meters).     |
///   | bool       | "gravityCompensation" | Enable or disable hardware gravity compensation.     |
///
/// \sa NovintHapticDevice
/// \sa NovintCommonDevice, SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class Novint7DofDevice : public NovintCommonDevice
{
public:
	/// Constructor.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	/// \param initializationName The name passed to HDAL when initializing the device.  This should match a
	/// 	configured Novint device; alternately, an empty string indicates the default device.
	Novint7DofDevice(const std::string& uniqueName, const std::string& initializationName);

	/// Destructor.
	virtual ~Novint7DofDevice();

private:
	bool is7DofDevice() const override;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_NOVINT_NOVINT7DOFDEVICE_H

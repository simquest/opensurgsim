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

#include "SurgSim/Devices/Novint/NovintCommonDevice.h"

namespace SurgSim
{
namespace Device
{


/// A class implementing the communication with a Novint Falcon device.
///
/// This should provide basic support for any device that can communicate using the Novint HDAL SDK toolkit, such as
/// the off-the-shelf Novint Falcon haptic gaming controller.  Note that certain devices may require device-specific
/// support in the code to enable particular hardware features.  In particular, the Novint Falcon with the Open Surgery
/// Grip will not be able to produce torques unless it is accessed using the Novint7DofHapticDevice class.
///
/// \par Application input provided by the device:
///   | type       | name        |                                                                |
///   | ----       | ----        | ---                                                            |
///   | pose       | "pose"      | %Device pose (units are meters).                               |
///   | bool       | "button1"   | %State of the first device button if present.                  |
///   | bool       | "button2"   | %State of the second device button if present.                 |
///   | bool       | "button3"   | %State of the third device button if present.                  |
///   | bool       | "button4"   | %State of the third device button if present.                  |
///   | bool       | "isHomed"   | %Device homing status.                                         |
/// Note that \c button1 through \c 4 correspond to the buttons 0 through 3 provided by the
/// HDAL SDK, but a custom Novint device might have fewer than 4 buttons.
///
/// \par Application output used by the device:
///   | type       | name                  |                                                      |
///   | ----       | ----                  | ---                                                  |
///   | vector     | "force"               | %Device output force (units are newtons).            |
///   | bool       | "gravityCompensation" | Enable or disable hardware gravity compensation.     |
///
/// \sa Novint7DofHapticDevice
/// \sa NovintCommonDevice, SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class NovintDevice : public NovintCommonDevice
{
public:
	/// Constructor.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	/// \param initializationName The name passed to HDAL when initializing the device.  This should match a
	/// 	configured Novint device; alternately, an empty string indicates the default device.
	NovintDevice(const std::string& uniqueName, const std::string& initializationName);

	/// Destructor.
	virtual ~NovintDevice();
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_NOVINT_NOVINTDEVICE_H

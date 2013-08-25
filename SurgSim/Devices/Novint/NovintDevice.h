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

#include <memory>
#include <string>

#include <SurgSim/Input/CommonDevice.h>

namespace SurgSim
{
namespace Device
{
class NovintScaffold;


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
///   | bool       | "button1"   | %State of the first device button.                             |
///   | bool       | "button2"   | %State of the second device button if present.                 |
///   | bool       | "button3"   | %State of the third device button (probably doesn't exist).    |
///   | bool       | "button4"   | %State of the third device button (probably doesn't exist).    |
///   | bool       | "isHomed"   | %Device homing status.                                         |
/// Note that \c button1 through \c 4 correspond to the buttons 0 through 3 provided by the
/// HDAL SDK, but a custom Novint device might have fewer than 4 buttons.
///
/// \par Application output used by the device:
///   | type       | name        |                                                                |
///   | ----       | ----        | ---                                                            |
///   | vector     | "force"     | %Device output force (units are newtons).                      |
///
/// \sa SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class NovintDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	/// \param initializationName The name passed to HDAL when initializing the device.  This should match a
	/// 	configured NOVINT device; alternately, an empty string indicates the default device.
	NovintDevice(const std::string& uniqueName, const std::string& initializationName);

	/// Destructor.
	virtual ~NovintDevice();

	/// Gets the name used by the Novint device configuration to refer to this device.
	/// Note that this may or may not be the same as the device name retrieved by getName().
	/// An empty string indicates the default device.
	/// \return	The initialization name.
	std::string getInitializationName() const;

	virtual bool initialize() override;

	virtual bool finalize() override;

	/// Check whether this device is initialized.
	bool isInitialized() const;

private:
	friend class NovintScaffold;

	std::shared_ptr<NovintScaffold> m_scaffold;
	std::string m_initializationName;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_NOVINT_NOVINTDEVICE_H

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

#ifndef SURGSIM_DEVICE_RAWMULTIAXISDEVICE_H
#define SURGSIM_DEVICE_RAWMULTIAXISDEVICE_H

#include <memory>
#include <string>

#include <SurgSim/Input/CommonDevice.h>

namespace SurgSim
{
namespace Device
{
class RawMultiAxisScaffold;


/// A class implementing the communication with a multi-axis controller input device, for example a 3DConnexion
/// SpaceNavigator.
///
/// This object will only generate raw output reported by the controller, which indicates the
/// movement of the controller from its rest state.  Normally, that result will need to be integrated to allow the
/// controller to be treated as a differential device, where holding the controller moves the pose and releasing
/// the controller lets the pose hold steady in its new state.  The MultiAxisDevice class provides that.
///
/// \par Application input provided by the device:
///   | type       | name              |                                                                           |
///   | ----       | ----              | ---                                                                       |
///   | pose       | "pose"            | %Absolute device pose (units are ill-defined, but nominally meters).      |
///   | bool       | "button1"         | True if button 1 exists and is pressed.                                   |
///   | bool       | "button2"         | True if button 2 exists and is pressed.                                   |
///   | bool       | "button3"         | True if button 3 exists and is pressed.                                   |
///   | bool       | "button4"         | True if button 4 exists and is pressed.                                   |
///
/// \par Application output used by the device:
///   | type       | name              |                                                                           |
///   | ----       | ----              | ---                                                                       |
///   | bool       | "led1"            | If the device has at least one LED light, controls the first one.         |
///
/// \sa MultiAxisDevice, SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class RawMultiAxisDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	explicit RawMultiAxisDevice(const std::string& uniqueName);

	/// Destructor.
	virtual ~RawMultiAxisDevice();

	virtual bool initialize() override;

	virtual bool finalize() override;

	/// Check wheter this device is initialized.
	bool isInitialized() const;

private:
	friend class RawMultiAxisScaffold;

	std::shared_ptr<RawMultiAxisScaffold> m_scaffold;
};

};  // namespace Device
};  // namespace SurgSim

#endif // SURGSIM_DEVICE_RAWMULTIAXISDEVICE_H

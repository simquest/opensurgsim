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

#ifndef SURGSIM_DEVICES_TRACKIR_TRACKIRDEVICE_H
#define SURGSIM_DEVICES_TRACKIR_TRACKIRDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Devices
{
class TrackIRScaffold;

SURGSIM_STATIC_REGISTRATION(TrackIRDevice);

/// A class implementing the communication with Natural Point TrackIR camera.
/// Z is the direction that the camera faces.
/// Y is in the direction of the camera's up.
/// X is the direction to the camera's left (making a right-hand coordinate system).
/// \par Application input provided by the device:
///   | type       | name              |                                        |
///   | ----       | ----              | ---                                    |
///   | pose       | "pose"            | %Device pose (units are meters).       |
///
/// \par Application output used by the device: none.
///
/// \sa SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class TrackIRDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	/// \param uniqueName A unique name for the device.
	explicit TrackIRDevice(const std::string& uniqueName);

	SURGSIM_CLASSNAME(SurgSim::Devices::TrackIRDevice);

	/// Destructor.
	virtual ~TrackIRDevice();

	bool initialize() override;

	bool isInitialized() const override;

	/// Sets the position scale for this device.
	/// The position scale controls how much the pose changes for a given device translation.
	/// The default value for a raw device tries to correspond to the actual physical motion of the device.
	void setPositionScale(double scale);
	/// Gets the position scale for this device.
	double getPositionScale() const;

	/// Sets the orientation scale for this device.
	/// The orientation scale controls how much the pose changes for a given device rotation.
	/// The default value for a raw device tries to correspond to the actual physical motion of the device.
	void setOrientationScale(double scale);
	/// Gets the orientation scale for this device.
	double getOrientationScale() const;

private:
	friend class TrackIRScaffold;

	bool finalize() override;

	// Returns the default position scale
	static double defaultPositionScale();
	// Returns the default rotation scale
	static double defaultOrientationScale();

	/// Scale factor for the position axes; stored locally before the device is initialized.
	double m_positionScale;
	/// Scale factor for the orientation axes; stored locally before the device is initialized.
	double m_orientationScale;

	/// Communication with hardware is handled by scaffold.
	std::shared_ptr<TrackIRScaffold> m_scaffold;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_TRACKIR_TRACKIRDEVICE_H

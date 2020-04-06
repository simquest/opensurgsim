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

#ifndef SURGSIM_DEVICES_MULTIAXIS_MULTIAXISDEVICE_H
#define SURGSIM_DEVICES_MULTIAXIS_MULTIAXISDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Devices/DeviceFilters/FilteredDevice.h"
#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Devices
{

class RawMultiAxisDevice;
class PoseIntegrator;

SURGSIM_STATIC_REGISTRATION(MultiAxisDevice);

/// A class implementing the communication with a multi-axis controller input device, for example a 3DConnexion
/// SpaceNavigator.
///
/// This object will integrate the output of the physical device, treating it as a differential device.  In other
/// words, holding the controller moves the pose and releasing the controller lets the pose hold steady in its
/// new state.
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
///   | pose       | "pose"            | This resets the builtin integrator to the give pose                       | 
///
/// \sa RawMultiAxisDevice
class MultiAxisDevice : public FilteredDevice
{
public:
	/// Constructor.
	/// \param uniqueName A unique name for the device that will be used by the application.
	explicit MultiAxisDevice(const std::string& uniqueName);

	SURGSIM_CLASSNAME(SurgSim::Devices::MultiAxisDevice);

	/// Sets the position scale for this device.
	/// The position scale controls how much the pose changes for a given device translation.
	/// The default value for a raw device tries to correspond to the actual physical motion of the device.
	void setPositionScale(double scale);

	/// Gets the position scale for this device.
	/// \return The position scale.
	double getPositionScale() const;

	/// Sets the orientation scale for this device.
	/// The orientation scale controls how much the pose changes for a given device rotation.
	/// \param scale The new scale.
	/// \note The default value for a raw device tries to correspond to the actual physical motion of the device.
	void setOrientationScale(double scale);

	/// Gets the orientation scale for this device.
	/// \return The orientation scale.
	double getOrientationScale() const;

	/// Turns on or off the axis dominance setting for this device.
	/// When axis dominance is on, only one (the largest) of the 6 pure axis directions is allowed to be active.
	/// In other words, the device will be translating in X, or in Y, or in Z, or rotating around X, or around Y,
	/// or around Z; but only one of those at a time.
	/// \param onOff Whether or not to use only the dominant axis.
	void setAxisDominance(bool onOff);

	/// Gets the axis dominance setting for this device.
	/// \return True if using axis dominance.
	bool isUsingAxisDominance() const;

	/// Sets the string name of the boolean entry that will reset the pose to its initial value.
	/// \param name The name of the NamedData<bool> entry, e.g., SurgSim::DataStructures::Names::BUTTON_1.
	/// \sa PoseIntegrator::setReset
	void setReset(const std::string& name);

private:
	/// Get the default position scale from device ticks to meters.
	/// \return The default position scale, in meters per tick.
	static double defaultPositionScale();

	/// Get the default rotation from device ticks to radians.
	/// \return The default rotation scale, in radians per tick.
	static double defaultOrientationScale();

	/// The raw underlying device.
	std::shared_ptr<RawMultiAxisDevice> m_rawDevice;

	/// The pose integration filter.
	std::shared_ptr<PoseIntegrator> m_filter;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_MULTIAXISDEVICE_H

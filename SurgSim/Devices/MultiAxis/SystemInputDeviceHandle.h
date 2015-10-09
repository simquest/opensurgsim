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

#ifndef SURGSIM_DEVICES_MULTIAXIS_SYSTEMINPUTDEVICEHANDLE_H
#define SURGSIM_DEVICES_MULTIAXIS_SYSTEMINPUTDEVICEHANDLE_H

#include <string>
#include <memory>
#include <array>
#include <vector>

namespace SurgSim
{
namespace Devices
{

/// A wrapper for system-dependent access to an input/HID device.
class SystemInputDeviceHandle
{
public:
	/// The maximum number of axes supported by any device object.
	static const size_t MAX_NUM_AXES = 6;
	/// The maximum number of buttons supported by any device object.
	static const size_t MAX_NUM_BUTTONS = 4;

	/// Type used to store axis states.
	typedef std::array<int, MAX_NUM_AXES> AxisStates;
	/// Type used to store button states.
	typedef std::array<bool, MAX_NUM_BUTTONS> ButtonStates;


	/// Destructor.
	virtual ~SystemInputDeviceHandle();

	/// Gets the name returned by the operating system for this device.
	/// \return	The reported name, or "???" if no name information could be found.
	virtual std::string getDeviceName() const = 0;

	/// Gets the device identifiers.
	/// \param [out] vendorId	The USB or PCI vendor identifier.
	/// \param [out] productId	The USB or PCI product identifier.
	/// \return	true if it succeeds.
	virtual bool getDeviceIds(int* vendorId, int* productId) const = 0;

	/// Queries if this device has 3 translation and 3 rotation axes.
	/// \return	true if the desired axes are present.
	virtual bool hasTranslationAndRotationAxes() const = 0;

	/// Updates the axis and states from the device input, if any.
	/// \param [in,out] axisStates	The states for each axis of the device.
	/// \param [in,out] buttonStates	The states for each device button.
	/// \param [out] updated True if any states were actually updated.  (Note that even if this value is true, the
	/// 	states may not have changed value; one or more states could have been updated to the same value.)
	/// \return	true if the operation was successful; false if the device is no longer in a usable state.
	virtual bool updateStates(AxisStates* axisStates, ButtonStates* buttonStates, bool* updated) = 0;

	/// Prepares the handle for sampling thread shutdown.
	/// Should be called from the same thread that was calling updateStates, after the calls to updateStates
	/// have been stopped, but before object destruction.
	virtual void prepareForShutdown();

protected:
	/// Default constructor.
	/// Cannot be called directly; see open and enumerate.
	SystemInputDeviceHandle();

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	SystemInputDeviceHandle(const SystemInputDeviceHandle& other) /*= delete*/;
	SystemInputDeviceHandle& operator=(const SystemInputDeviceHandle& other) /*= delete*/;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_SYSTEMINPUTDEVICEHANDLE_H

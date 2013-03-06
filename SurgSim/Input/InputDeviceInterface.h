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

#ifndef SURGSIM_INPUT_INPUT_DEVICE_INTERFACE_H
#define SURGSIM_INPUT_INPUT_DEVICE_INTERFACE_H

#include <memory>
#include <string>

#include <SurgSim/Input/InputDeviceListenerInterface.h>

namespace SurgSim
{
namespace Input
{

/// Interface used to communicate with user-interface hardware devices.
///
/// Note that despite the interface name, many devices that implement it will
/// not be input-only.  Haptic devices are the most obvious example, but
/// many other devices can, for example, display visual indicators such as
/// LEDs, etc.
///
/// Derived classes will likely want to hide their constructor and only
/// allow creation through a manager object for that type of device.
class InputDeviceInterface
{
public:
	/// Virtual destructor (empty).
	virtual ~InputDeviceInterface() {};

	/// Return a (hopefully unique) device name.
	virtual std::string getName() const = 0;

	/// Adds a listener that will be notified when input state is updated, and asked for output state when
	/// needed.
	///
	/// \param listener The listener to be added.
	///
	/// \return true on success, false on failure.
	virtual bool addListener(std::shared_ptr<InputDeviceListenerInterface> listener) = 0;

	/// Adds a listener that will be notified when input state is updated.
	/// The listener will not be used to provide output.
	///
	/// \param listener The listener to be added.
	/// \return true on success, false on failure.
	virtual bool addInputListener(std::shared_ptr<InputDeviceListenerInterface> listener) = 0;

	/// Removes a listener previously added via \ref addListener or \ref addInputListener.
	virtual bool removeListener(std::shared_ptr<InputDeviceListenerInterface> listener) = 0;

protected:

	/// Fully initialize the device.
	///
	/// When the manager object creates the device, the internal state of the device usually isn't fully
	/// initialized yet.  This method performs any needed initialization.
	virtual bool initialize() = 0;

	/// Finalize (de-initialize) the device.
	virtual bool finalize() = 0;
};

};  // namespace Input
};  // namespace SurgSim

#endif // SURGSIM_INPUT_INPUT_DEVICE_INTERFACE_H

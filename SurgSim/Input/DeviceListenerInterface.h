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

#ifndef SURGSIM_INPUT_DEVICE_LISTENER_INTERFACE_H
#define SURGSIM_INPUT_DEVICE_LISTENER_INTERFACE_H

#include <string>

namespace SurgSim
{
namespace DataStructures
{
class DataGroup;
}; // namespace DataStructures

namespace Input
{

class DeviceInterface;


/// Interface for a listener that monitors device and signal state updates.
class DeviceListenerInterface
{
public:
	/// Virtual destructor (empty).
	virtual ~DeviceListenerInterface() {};

	/// Notifies the listener that the application input coming from the device has been updated.
	///
	/// Typical input data contents (but note that individual devices may do things differently):
	///   | type       | name        |                                                                |
	///   | ----       | ----        | ---                                                            |
	///   | pose       | "pose"      | %Device pose (units are meters).                                |
	///   | bool       | "button0"   | %Device button 0 state.                                         |
	///   | bool       | "button1"   | %Device button 1 state.                                         |
	///
	/// Other possible contents includes:
	///   | type       | name        |                                                                |
	///   | ----       | ----        |                                                                |
	///   | bool       | "isHomed"   | %Device homing status.                                          |
	///   | bool       | "isHomed0"  | Individual axis homing status, etc.                            |
	///   | bool       | "isHeld"    | Safety sensor etc. status.                                     |
	///   | string     | "model"     | %Device model description.                                      |
	///   | string     | "serial"    | Serial number string.                                          |
	///   | (any)      | "debug:*"   | Various debugging information                                  |
	///
	/// \param device The name of the device that is producing the input.  This should only be used to identify
	/// 	the device (e.g. if the listener is listening to several devices at once).
	/// \param inputData The input state coming from the device (i.e. the application's input but the device's
	/// 	<em>output</em>).
	virtual void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) = 0;

	/// Asks the listener to provide output state to the device.
	///
	/// Note that devices may never call this method, e.g. because the listener was registered via \ref
	/// DeviceInterface::addInputListener, or because the device doesn't have any output capability, or
	/// because another listener is already providing output information.
	///
	///  Typical output data contents (but note that individual devices may do things differently):
	///   | type       | name        |                                                                |
	///   | ----       | ----        | ---                                                            |
	///   | vector     | "force"     | Commanded force for the device (units are newtons).            |
	///   | vector     | "torque"    | Commanded torque for the device (units are newton-meters).     |
	///   | bool       | "isEnabled" | Safety switch input.                                           |
	///
	/// Other possible contents includes:
	///   | type       | name        |                                                                |
	///   | ----       | ----        | ---                                                            |
	///   | bool       | "led0"      | Desired state for LED 0.                                       |
	///   | bool       | "led1"      | Desired state for LED 1.                                       |
	///   | string     | "toolId"    | Calibration ID to use, e.g. for camera devices.                |
	///
	/// \param device The name of the device that is requesting the output.  This should only be used to identify
	/// 	the device (e.g. if the listener is listening to several devices at once).
	/// \param [out] outputData The output state being fed into the device (i.e. the application's output but
	/// 	the device's <em>input</em>).
	///
	/// \return true if the listener has provided some output, false if it refuses to do so.  In general, it is
	/// 		a good idea to register a listener than never provides output using \ref
	/// 		DeviceInterface::addInputListener, so that this method will never be called.  Any listener
	/// 		that returns false should leave outputData unmodified.  This allows the application to register
	/// 		several listeners with the same device, but ensure that only one of them will be providing the
	/// 		output information.
	virtual bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) = 0;
};

};  // namespace Input
};  // namespace SurgSim

#endif // SURGSIM_INPUT_DEVICE_LISTENER_INTERFACE_H

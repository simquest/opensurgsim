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

#ifndef SURGSIM_INPUT_INPUTCONSUMERINTERFACE_H
#define SURGSIM_INPUT_INPUTCONSUMERINTERFACE_H

#include <string>

namespace SurgSim
{
namespace DataStructures
{
class DataGroup;
}; // namespace DataStructures

namespace Input
{


/// Interface for a consumer that monitors device and signal state updates (pose, buttons, etc).
class InputConsumerInterface
{
public:
	/// Virtual destructor (empty).
	virtual ~InputConsumerInterface()
	{
	}

	/// Notifies the consumer that the application input coming from the device has been updated.
	///
	/// Typical input data contents (but note that individual devices may do things differently):
	///   | type       | name        |                                                                |
	///   | ----       | ----        | ---                                                            |
	///   | pose       | "pose"      | %Device pose (units are meters).                               |
	///   | bool       | "button1"   | State of the first device button.                              |
	///   | bool       | "button2"   | State of the second device button (and so on).                 |
	///
	/// Other possible contents includes:
	///   | type       | name        |                                                                |
	///   | ----       | ----        |                                                                |
	///   | bool       | "isHomed"   | %Device homing status.                                         |
	///   | bool       | "isHomedX"  | Individual homing status for the X axis (and so on).           |
	///   | bool       | "isHeld"    | Safety sensor etc. status.                                     |
	///   | string     | "model"     | %Device model description.                                     |
	///   | string     | "serial"    | Serial number string.                                          |
	///   | (any)      | "debug:*"   | Various debugging information                                  |
	///
	/// \param device The name of the device that is producing the input.  This should only be used to identify
	/// 	the device (e.g. if the consumer is listening to several devices at once).
	/// \param inputData The application input state coming from the device.
	virtual void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) = 0;

	/// Set the initial input data group.
	/// \param device The name of the device that is producing the input.  This should only be used to identify
	/// 	the device (e.g. if the consumer is listening to several devices at once).
	/// \param inputData The application input state coming from the device.
	virtual void initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) = 0;
};


};  // namespace Input
};  // namespace SurgSim

#endif // SURGSIM_INPUT_INPUTCONSUMERINTERFACE_H

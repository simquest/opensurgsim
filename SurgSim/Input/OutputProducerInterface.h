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

#ifndef SURGSIM_INPUT_OUTPUTPRODUCERINTERFACE_H
#define SURGSIM_INPUT_OUTPUTPRODUCERINTERFACE_H

#include <string>

namespace SurgSim
{
namespace DataStructures
{
class DataGroup;
}; // namespace DataStructures

namespace Input
{


/// Interface for a producer that generates device output updates (forces, status LED state, etc).
class OutputProducerInterface
{
public:
	/// Virtual destructor (empty).
	virtual ~OutputProducerInterface()
	{
	}

	/// Asks the producer to provide output state to the device.
	///
	/// Note that devices may never call this method, e.g. because the device doesn't actually have any
	/// output capability.
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
	/// 	the device (e.g. if the producer is listening to several devices at once).
	/// \param [out] outputData The application output state being fed into the device.
	///
	/// \return true if the producer has provided some output, false if it refuses to do so.  A producer
	/// 		that returns false should leave outputData unmodified.
	virtual bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) = 0;
};


};  // namespace Input
};  // namespace SurgSim

#endif // SURGSIM_INPUT_OUTPUTPRODUCERINTERFACE_H

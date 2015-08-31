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

#include <memory>

#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Devices/TrackIR/TrackIRDevice.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"

#include "SurgSim/Testing/VisualTestCommon/ToolSquareTest.h"

using SurgSim::Input::DeviceInterface;
using SurgSim::Devices::TrackIRDevice;
using SurgSim::Devices::IdentityPoseDevice;


int main(int argc, char** argv)
{
	std::shared_ptr<TrackIRDevice> toolDevice = std::make_shared<TrackIRDevice>("TrackIRDevice");

	// The square is controlled by a second device.  For a simple test, we're using an IdentityPoseDevice--
	// a pretend device that doesn't actually move.
	std::shared_ptr<DeviceInterface> squareDevice = std::make_shared<IdentityPoseDevice>("IdentityPoseDevice");

	runToolSquareTest(toolDevice, squareDevice,
					  //2345678901234567890123456789012345678901234567890123456789012345678901234567890
					  "Move the TrackIR device to move the sphere tool.");

	std::cout << std::endl << "Exiting." << std::endl;
	// Cleanup and shutdown will happen automatically as objects go out of scope.

	return 0;
}

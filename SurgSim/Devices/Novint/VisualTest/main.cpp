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
#include "SurgSim/Devices/Novint/NovintDevice.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"

#include "SurgSim/Testing/VisualTestCommon/ToolSquareTest.h"

using SurgSim::Input::DeviceInterface;
using SurgSim::Device::NovintDevice;
using SurgSim::Device::IdentityPoseDevice;


// Define the HDAL name of the device to use.
static const char* const NOVINT_DEVICE_NAME = ""; // An empty name will instantiate the default falcon.
//static const char* const NOVINT_DEVICE_NAME = "FALCON_HTHR_R";
//static const char* const NOVINT_DEVICE_NAME = "FALCON_FRANKEN_L";
//static const char* const NOVINT_DEVICE_NAME = "FALCON_BURRv3_1";
//static const char* const NOVINT_DEVICE_NAME = "FALCON_BURRv3_2";


int main(int argc, char** argv)
{
	std::shared_ptr<NovintDevice> toolDevice = std::make_shared<NovintDevice>("NovintDevice", NOVINT_DEVICE_NAME);

	// The square is controlled by a second device.  For a simple test, we're using an IdentityPoseDevice--
	// a pretend device that doesn't actually move.
	std::shared_ptr<DeviceInterface> squareDevice = std::make_shared<IdentityPoseDevice>("IdentityPoseDevice");

	runToolSquareTest(toolDevice, squareDevice,
					  //2345678901234567890123456789012345678901234567890123456789012345678901234567890
					  "Move the Novint Falcon device to move the sphere tool.");

	std::cout << std::endl << "Exiting." << std::endl;
	// Cleanup and shutdown will happen automatically as objects go out of scope.

	return 0;
}

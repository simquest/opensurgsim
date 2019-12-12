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

#include <iostream>
#include <memory>

#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Devices/Novint/NovintDevice.h"
#include "SurgSim/Testing/VisualTestCommon/ToolSquareTest.h"

// The initialization name of the NovintDevice.  An empty string will use the first available Falcon.
static const char* const NOVINT_DEVICE_NAME = "";
// true if the Novint should be 7Dof, false if the Novint should be 3Dof.
static const bool NOVINT_7_DOF = false;

int main(int argc, char** argv)
{
	auto toolDevice = std::make_shared<SurgSim::Devices::NovintDevice>("NovintDevice");
	toolDevice->setInitializationName(NOVINT_DEVICE_NAME);
	toolDevice->set7DofDevice(NOVINT_7_DOF);
	auto squareDevice = std::make_shared<SurgSim::Devices::IdentityPoseDevice>("IdentityPoseDevice");

	runToolSquareTest(toolDevice, squareDevice, "Move the Novint Falcon device to move the sphere tool.");

	std::cout << std::endl << "Exiting." << std::endl;
	// Cleanup and shutdown will happen automatically as objects go out of scope.

	return 0;
}

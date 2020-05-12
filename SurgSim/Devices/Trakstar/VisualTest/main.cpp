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

#include "SurgSim/Devices/Trakstar/TrakstarDevice.h"
#include "SurgSim/Testing/VisualTestCommon/ToolSquareTest.h"

using SurgSim::Devices::TrakstarDevice;

int main(int argc, char** argv)
{
	auto toolDevice = std::make_shared<TrakstarDevice>("TrakstarDevice1");
	toolDevice->setSensorId((unsigned short)(0));

	auto squareDevice = std::make_shared<TrakstarDevice>("TrakstarDevice2");
	toolDevice->setSensorId((unsigned short)(1));

	runToolSquareTest(toolDevice, squareDevice, "Move the devices to move the sphere and square.");

	std::cout << std::endl << "Exiting." << std::endl;
	// Cleanup and shutdown will happen automatically as objects go out of scope.

	return 0;
}

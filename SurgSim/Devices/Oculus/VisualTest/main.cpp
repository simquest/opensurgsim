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

#include <SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h>
#include <SurgSim/Devices/Oculus/OculusDevice.h>
#include <SurgSim/Input/DeviceInterface.h>
#include <SurgSim/Testing/VisualTestCommon/ToolSquareTest.h>

using SurgSim::Device::IdentityPoseDevice;
using SurgSim::Device::OculusDevice;
using SurgSim::Input::DeviceInterface;

int main(int argc, char** argv)
{
	auto toolDevice = std::make_shared<OculusDevice>("OculusDevice");

	// The square is controlled by a second device.  For a simple test, we're using an IdentityPoseDevice--
	// a pretend device that doesn't actually move.
	std::shared_ptr<DeviceInterface> squareDevice = std::make_shared<IdentityPoseDevice>("IdentityPoseDevice");

	runToolSquareTest(toolDevice, squareDevice, "Move the Oculus device to move the sphere tool.");

	std::cout << std::endl << "Exiting." << std::endl;
	// Cleanup and shutdown will happen automatically as objects go out of scope.

	return 0;
}

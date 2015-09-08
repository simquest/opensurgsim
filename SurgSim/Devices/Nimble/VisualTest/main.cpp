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
#include "SurgSim/Devices/DeviceFilters/PoseTransform.h"
#include "SurgSim/Devices/Nimble/NimbleDevice.h"

#include "SurgSim/Testing/VisualTestCommon/ToolSquareTest.h"

using SurgSim::Input::DeviceInterface;
using SurgSim::Devices::PoseTransform;
using SurgSim::Devices::NimbleDevice;

int main(int argc, char** argv)
{
	auto leftDevice = std::make_shared<NimbleDevice>("NimbleDeviceLeft");
	auto leftDeviceFilter = std::make_shared<PoseTransform>("NimbleDeviceLeftDeviceFilter");
	leftDeviceFilter->setTranslationScale(0.0001);
	leftDevice->addInputConsumer(leftDeviceFilter);
	leftDevice->setOutputProducer(leftDeviceFilter);
	leftDevice->initialize();

	auto rightDevice = std::make_shared<NimbleDevice>("NimbleDeviceRight");
	rightDevice->setupToTrackRightHand();
	auto rightDeviceFilter = std::make_shared<PoseTransform>("NimbleDeviceRightDeviceFilter");
	rightDeviceFilter->setTranslationScale(0.0001);
	rightDevice->addInputConsumer(rightDeviceFilter);
	rightDevice->setOutputProducer(rightDeviceFilter);
	rightDevice->initialize();

	runToolSquareTest(leftDeviceFilter, rightDeviceFilter, "Move the hands to control the sphere/square.");

	std::cout << std::endl << "Exiting." << std::endl;
	// Cleanup and shutdown will happen automatically as objects go out of scope.

	return 0;
}

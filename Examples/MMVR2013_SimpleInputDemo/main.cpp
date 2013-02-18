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

#include <SurgSim/Devices/Phantom/PhantomManager.h>
#include <SurgSim/Devices/Phantom/PhantomDevice.h>

using SurgSim::Device::PhantomManager;
using SurgSim::Device::PhantomDevice;

#include "SimpleSquareForce.h"


int main(int argc, char** argv)
{
	std::shared_ptr<PhantomManager> phantomDeviceManager = std::make_shared<PhantomManager>();
	std::shared_ptr<PhantomDevice> device = phantomDeviceManager->createDevice("Phantom1", "Default PHANToM");
	if (! device)
	{
		printf("--- Press Enter to quit the application! ---\n");
		getc(stdin);
		return -1;
	}

	std::shared_ptr<SimpleSquareForce> squareForce = std::make_shared<SimpleSquareForce>();
	device->addListener(squareForce);

	printf("\n"
	       "**********************************************************************\n"
	       "Move the Phantom up / down to feel a square in the horizontal plane.\n"
	       "\n"
	       "When done, press Enter to quit the application.\n"
	       "**********************************************************************\n");

	getc(stdin);

	// Cleanup and shutdown the haptic device, cleanup all callbacks.

	return 0;
}

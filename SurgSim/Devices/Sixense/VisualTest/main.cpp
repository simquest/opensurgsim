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
#include <GL/glut.h>

#include <SurgSim/Devices/Sixense/SixenseDevice.h>
#include <SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h>

using SurgSim::Device::SixenseDevice;
using SurgSim::Device::IdentityPoseDevice;

#include "MovingSquareForce.h"
#include "MovingSquareGlutWindow.h"


int main(int argc, char** argv)
{

	std::shared_ptr<SixenseDevice> toolDevice = std::make_shared<SixenseDevice>("ToolDevice");
	if (! toolDevice || ! toolDevice->initialize())
	{
		printf("--- Press Enter to quit the application! ---\n");
		getc(stdin);
		return -1;
	}

	// The square is controlled by a second device. Unfortunately, we don't have a second hardware device yet, so
	// we're using an IdentityPoseDevice-- a pretend device that doesn't actually move.
	std::shared_ptr<IdentityPoseDevice> squareDevice = std::make_shared<IdentityPoseDevice>("SquareDevice");
	if (! squareDevice)
	{
		printf("--- Press Enter to quit the application! ---\n");
		getc(stdin);
		return -1;
	}
	std::shared_ptr<MovingSquareForce> squareForce =
		std::make_shared<MovingSquareForce>(toolDevice->getName(), squareDevice->getName());
	toolDevice->addInputConsumer(squareForce);
	toolDevice->setOutputProducer(squareForce);
	squareDevice->addInputConsumer(squareForce);
	// NB: the code does not currently support exerting the reaction force on the square.

	std::shared_ptr<MovingSquareGlutWindow> squareGlutWindow =
		std::make_shared<MovingSquareGlutWindow>(toolDevice->getName(), squareDevice->getName());
	toolDevice->addInputConsumer(squareGlutWindow);
	squareDevice->addInputConsumer(squareGlutWindow);

	printf("\n"
	       "**********************************************************************\n"
	       "Move the device up / down.  If haptic feedback is available,\n"
		   "you will feel a square in the horizontal plane.\n"
	       "\n"
	       "When done, press Enter to quit the application.\n"
	       "**********************************************************************\n");

	getc(stdin);

	// Cleanup and shutdown the haptic device, cleanup all callbacks.

	return 0;
}

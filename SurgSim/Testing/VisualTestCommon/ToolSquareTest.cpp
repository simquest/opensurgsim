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

#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Framework/Assert.h"

using SurgSim::Input::DeviceInterface;

#include "SurgSim/Testing/VisualTestCommon/MovingSquareForce.h"
#include "SurgSim/Testing/VisualTestCommon/MovingSquareGlutWindow.h"


void runToolSquareTest(std::shared_ptr<DeviceInterface> toolDevice, std::shared_ptr<DeviceInterface> squareDevice,
					   const char* testDescriptionMessage)
{
	SURGSIM_ASSERT(toolDevice && squareDevice);
	if (! toolDevice->initialize())
	{
		printf("Could not initialize device '%s' for the tool.\n"
			   "--- Press Enter to quit the application! ---\n", toolDevice->getName().c_str());
		getc(stdin);
		return;
	}
	if (! squareDevice->initialize())
	{
		printf("Could not initialize device '%s' for the square.\n"
			   "--- Press Enter to quit the application! ---\n", squareDevice->getName().c_str());
		getc(stdin);
		return;
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
		   "%s\n"
		   "\n"
		   "When done, press Enter to quit the application.\n"
		   "**********************************************************************\n",
		   testDescriptionMessage);

	// Wait for a key; the display, force generation, etc. all happen in separate threads.
	getc(stdin);

	toolDevice->removeInputConsumer(squareForce);
	toolDevice->removeOutputProducer(squareForce);
	squareDevice->removeInputConsumer(squareForce);

	toolDevice->removeInputConsumer(squareGlutWindow);
	squareDevice->removeInputConsumer(squareGlutWindow);

	// Right now you can't just tear down the MovingSquareGlutWindow, so we exit with prejudice instead.
	exit(0);
}

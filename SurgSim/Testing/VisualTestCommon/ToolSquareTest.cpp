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
#include <iostream>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#define GLUT_NO_LIB_PRAGMA 1
#include <GL/glut.h>
#endif

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
		std::cout << std::endl << "Could not initialize device named '" << toolDevice->getName() <<
			"' for the tool." << std::endl << "--- Press Enter to quit the application! ---" << std::endl;
		getc(stdin);
		return;
	}
	if (! squareDevice->initialize())
	{
		std::cout << std::endl << "Could not initialize device named '" << squareDevice->getName() <<
			"' for the square." << std::endl << "--- Press Enter to quit the application! ---" << std::endl;
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

	std::cout << std::endl << "**********************************************************************" << std::endl <<
		   testDescriptionMessage << std::endl << std::endl << "When done, press Enter to quit the application." <<
		   std::endl << "**********************************************************************" << std::endl;

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

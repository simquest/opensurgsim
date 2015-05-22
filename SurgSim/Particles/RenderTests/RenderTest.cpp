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

#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Particles/RenderTests/RenderTest.h"

namespace SurgSim
{
namespace Particles
{

void RenderTests::SetUp()
{
	runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	runtime->addManager(graphicsManager);

	physicsManager = std::make_shared<SurgSim::Physics::PhysicsManager>();
	runtime->addManager(physicsManager);

	behaviorManager = std::make_shared<SurgSim::Framework::BehaviorManager>();
	runtime->addManager(behaviorManager);

	scene = runtime->getScene();

	viewElement = std::make_shared<SurgSim::Graphics::OsgViewElement>("Physics Render Scene");
	scene->addSceneElement(viewElement);
}

void RenderTests::TearDown()
{
	runtime->stop();
}

void RenderTests::runTest(const SurgSim::Math::Vector3d& cameraPosition, const SurgSim::Math::Vector3d& cameraLookAt,
						  double miliseconds)
{
	using SurgSim::Graphics::OsgAxesRepresentation;

	viewElement->enableManipulator(true);
	viewElement->setManipulatorParameters(cameraPosition, cameraLookAt);

	std::shared_ptr<OsgAxesRepresentation> axes = std::make_shared<OsgAxesRepresentation>("axes");
	axes->setSize(1.0);
	viewElement->addComponent(axes);

	/// Run the thread
	runtime->start();

	boost::this_thread::sleep(boost::posix_time::milliseconds(miliseconds));
}

}; // namespace Particles
}; // namespace SurgSim

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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"

#include <SurgSim/Blocks/BasicSceneElement.h>
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"

/// Create a SceneElement with stapler data load from obj file
std::shared_ptr<SurgSim::Framework::SceneElement> createStapler()
{
	std::shared_ptr<SurgSim::Framework::SceneElement> element;
	element = std::make_shared<SurgSim::Blocks::BasicSceneElement>("Stapler");

	std::shared_ptr<SurgSim::Graphics::SceneryRepresentation> stapler;
	stapler = std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>("Stapler");
	stapler->setFileName("stapler_collision.obj");

	element->addComponent(stapler);
	
	return element;
}

/// Create a SceneElement with arm data load from obj file
std::shared_ptr<SurgSim::Framework::SceneElement> createArm()
{
	std::shared_ptr<SurgSim::Framework::SceneElement> element;
	element = std::make_shared<SurgSim::Blocks::BasicSceneElement>("Arm");

	std::shared_ptr<SurgSim::Graphics::SceneryRepresentation> arm;
	arm = std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>("Arm");
	arm->setFileName("arm_collision.obj");

	element->addComponent(arm);

	return element;
}

std::shared_ptr<SurgSim::Graphics::ViewElement> createView()
{
	std::shared_ptr<SurgSim::Graphics::OsgViewElement> view;
	view = std::make_shared<SurgSim::Graphics::OsgViewElement>("StaplingDemoView");
	view->enableManipulator(true);
	view->setManipulatorParameters(SurgSim::Math::Vector3d(0.0, 0.5, 1.0), SurgSim::Math::Vector3d(0.0, 0.5, 0.5));
	return view;
}

int main(int argc, char* argv[])
{
	// Create managers
	std::shared_ptr<SurgSim::Graphics::Manager> graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	// Scene will contain all SceneElements in this stapler demo.
	std::shared_ptr<SurgSim::Framework::Scene> scene = runtime->getScene(); 

	// Load scenery objects into Scene.
	scene->addSceneElement(createStapler());
	scene->addSceneElement(createArm());

	scene->addSceneElement(createView());

	runtime->addManager(graphicsManager);

	runtime->execute();

	return 0;
}

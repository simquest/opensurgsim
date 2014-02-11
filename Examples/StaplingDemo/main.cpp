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
#include <string>

#include "SurgSim/Blocks/BasicSceneElement.h"
#include "SurgSim/Blocks/TransferInputPoseBehavior.h"
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"

#include "Examples/StaplingDemo/AddStapleBehavior.h"

/// Create a SceneElement with stapler data load from obj file
std::shared_ptr<SurgSim::Framework::SceneElement> loadStapler(const std::string& fileName)
{
	std::shared_ptr<SurgSim::Graphics::SceneryRepresentation> stapler =
		std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>("Stapler");
	stapler->setFileName(fileName);

	auto inputComponent = std::make_shared<SurgSim::Input::InputComponent>("input");
	inputComponent->setDeviceName("MultiAxisDevice");

	auto transferInputPose = std::make_shared<SurgSim::Blocks::TransferInputPoseBehavior>("Input to scenery object");
	transferInputPose->setPoseSender(inputComponent);
	transferInputPose->setPoseReceiver(stapler);

	std::shared_ptr<SurgSim::Framework::SceneElement> element =
		std::make_shared<SurgSim::Blocks::BasicSceneElement>("SceneElement");
	element->addComponent(inputComponent);
	element->addComponent(stapler);
	element->addComponent(transferInputPose);

	auto addStaple = std::make_shared<AddStapleFromInputBehavior>("Staple");
	addStaple->setInputComponent(inputComponent);
	element->addComponent(addStaple);

	return element;
}

/// Create a SceneElement with arm data load from obj file
std::shared_ptr<SurgSim::Framework::SceneElement> loadArm(const std::string& fileName)
{
	std::shared_ptr<SurgSim::Framework::SceneElement> element =
		std::make_shared<SurgSim::Blocks::BasicSceneElement>("Arm");

	std::shared_ptr<SurgSim::Graphics::SceneryRepresentation> arm =
		std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>("Arm");

	arm->setFileName(fileName);
	element->addComponent(arm);

	return element;
}

std::shared_ptr<SurgSim::Graphics::ViewElement> createView()
{
	auto view = std::make_shared<SurgSim::Graphics::OsgViewElement>("StaplingDemoView");

	view->enableManipulator(true);
	view->setManipulatorParameters(SurgSim::Math::Vector3d(0.0, 0.5, 0.5), SurgSim::Math::Vector3d::Zero());
	return view;
}

int main(int argc, char* argv[])
{
	// Create managers
	auto behaviorManager = std::make_shared<SurgSim::Framework::BehaviorManager>();
	auto graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	auto physicsManager = std::make_shared<SurgSim::Physics::PhysicsManager>();
	auto inputManager = std::make_shared<SurgSim::Input::InputManager>();

	auto toolDevice = std::make_shared<SurgSim::Device::MultiAxisDevice>("MultiAxisDevice");
	SURGSIM_ASSERT(toolDevice->initialize() == true) <<
		"Could not initialize device '%s' for the tool.\n", toolDevice->getName().c_str();
	inputManager->addDevice(toolDevice);

	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	runtime->addManager(physicsManager);
	runtime->addManager(behaviorManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(inputManager);

	// Scene will contain all SceneElements in this stapler demo.
	std::shared_ptr<SurgSim::Framework::Scene> scene = runtime->getScene();
	scene->addSceneElement(loadStapler("stapler_collision.obj"));
	scene->addSceneElement(loadArm("forearm.osgb"));
	scene->addSceneElement(createView());

	runtime->execute();

	return 0;
}

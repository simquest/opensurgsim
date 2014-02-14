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
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "Examples/ExampleStapling/AddStapleBehavior.h"

/// Load scenery object from file
/// \param name Name of this scenery representation.
/// \param fileName Name of the file from which the scenery representation is loaded.
/// \return A SceneElement containing the scenery representation.
std::shared_ptr<SurgSim::Framework::SceneElement> createSceneryObject(const std::string& name,
																	  const std::string& fileName)
{
	auto sceneryRepresentation =
		std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>(name + "SceneryRepresentation");
	sceneryRepresentation->setFileName(fileName);

	auto sceneElement = std::make_shared<SurgSim::Blocks::BasicSceneElement>(name + "SceneElement");
	sceneElement->addComponent(sceneryRepresentation);

	return sceneElement;
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
	auto behaviorManager = std::make_shared<SurgSim::Framework::BehaviorManager>();
	auto graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	auto inputManager    = std::make_shared<SurgSim::Input::InputManager>();
	auto physicsManager  = std::make_shared<SurgSim::Physics::PhysicsManager>();

	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	runtime->addManager(behaviorManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(inputManager);
	runtime->addManager(physicsManager);

	auto toolDevice = std::make_shared<SurgSim::Device::MultiAxisDevice>("MultiAxisDevice");
	SURGSIM_ASSERT(toolDevice->initialize() == true) <<
		"Could not initialize device " << toolDevice->getName() << " for the tool.\n";
	inputManager->addDevice(toolDevice);

	std::shared_ptr<SurgSim::Framework::SceneElement> staplerSceneElement =
		createSceneryObject("stapler", "Geometry/stapler.obj");

	// In order to connect the stapler scenery representation to an input device, after create a SceneElement for it,
	// we need to retrieve the stapler scenery representation from the SceneElement.
	std::shared_ptr<SurgSim::Framework::Component> staplerComponent =
		staplerSceneElement->getComponent("staplerSceneryRepresentation");
	auto staplerSceneryRepresentation = std::static_pointer_cast<SurgSim::Framework::Representation>(staplerComponent);

	auto inputComponent = std::make_shared<SurgSim::Input::InputComponent>("input");
	inputComponent->setDeviceName("MultiAxisDevice");

	auto transferInputPose = std::make_shared<SurgSim::Blocks::TransferInputPoseBehavior>("Input to graghicalStapler");
	transferInputPose->setPoseSender(inputComponent);
	transferInputPose->setPoseReceiver(staplerSceneryRepresentation);

	auto addStapleBehavior = std::make_shared<AddStapleFromInputBehavior>("Staple");
	addStapleBehavior->setInputComponent(inputComponent);

	staplerSceneElement->addComponent(inputComponent);
	staplerSceneElement->addComponent(transferInputPose);
	staplerSceneElement->addComponent(addStapleBehavior);

	std::shared_ptr<SurgSim::Framework::Scene> scene = runtime->getScene();
	scene->addSceneElement(createSceneryObject("arm", "Geometry/forearm.osgb"));
	scene->addSceneElement(createView());
	scene->addSceneElement(staplerSceneElement);

	runtime->execute();

	return 0;
}

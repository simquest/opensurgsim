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
#include "SurgSim/Framework/TransferPropertiesBehavior.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "Examples/ExampleStapling/AddStapleBehavior.h"

using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Blocks::TransferInputPoseBehavior;
using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Framework::TransferPropertiesBehavior;
using SurgSim::Graphics::SceneryRepresentation;
using SurgSim::Graphics::ViewElement;
using SurgSim::Graphics::OsgManager;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::OsgSceneryRepresentation;
using SurgSim::Input::DeviceInterface;
using SurgSim::Input::InputComponent;
using SurgSim::Input::InputManager;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::PhysicsManager;

/// Load scenery object from file
/// \param name Name of this scenery representation.
/// \param fileName Name of the file from which the scenery representation is loaded.
/// \return A SceneElement containing the scenery representation.
std::shared_ptr<SceneElement> createSceneryObject(const std::string& name, const std::string& fileName)
{
	std::shared_ptr<SceneryRepresentation> sceneryRepresentation =
		std::make_shared<OsgSceneryRepresentation>(name + "SceneryRepresentation");
	sceneryRepresentation->setFileName(fileName);

	std::shared_ptr<SceneElement> sceneElement = std::make_shared<BasicSceneElement>(name + "SceneElement");
	sceneElement->addComponent(sceneryRepresentation);

	return sceneElement;
}

std::shared_ptr<SceneElement> createStapler(const std::string& staplerName, const std::string& deviceName)
{
	std::shared_ptr<SceneryRepresentation> staplerHandle =
		std::make_shared<OsgSceneryRepresentation>(staplerName + "Handle");
	std::shared_ptr<SceneryRepresentation> staplerIndicator =
		std::make_shared<OsgSceneryRepresentation>(staplerName + "Indicator");
	std::shared_ptr<SceneryRepresentation> staplerMarkings=
		std::make_shared<OsgSceneryRepresentation>(staplerName + "Markings");
	std::shared_ptr<SceneryRepresentation> staplerTrigger =
		std::make_shared<OsgSceneryRepresentation>(staplerName + "Trigger");

	staplerHandle->setFileName("Geometry/stapler_handle.obj");
	staplerIndicator->setFileName("Geometry/stapler_indicator.obj");
	staplerMarkings->setFileName("Geometry/stapler_markings.obj");
	staplerTrigger->setFileName("Geometry/stapler_trigger.obj");

	std::shared_ptr<TransferPropertiesBehavior> glue = std::make_shared<TransferPropertiesBehavior>("StaplerGlue");
	glue->connect(staplerHandle, "pose", staplerIndicator, "pose");
	glue->connect(staplerHandle, "pose", staplerMarkings, "pose");
	glue->connect(staplerHandle, "pose", staplerTrigger, "pose");

	std::shared_ptr<InputComponent> inputComponent = std::make_shared<InputComponent>("input");
	inputComponent->setDeviceName(deviceName);

	std::shared_ptr<TransferInputPoseBehavior> transferInputPose =
		std::make_shared<TransferInputPoseBehavior>("Input to graphicalStapler");
	transferInputPose->setPoseSender(inputComponent);
	transferInputPose->setPoseReceiver(staplerHandle);

	std::shared_ptr<AddStapleFromInputBehavior> addStapleBehavior =
		std::make_shared<AddStapleFromInputBehavior>("Staple");
	addStapleBehavior->setInputComponent(inputComponent);

	std::shared_ptr<SceneElement> sceneElement = std::make_shared<BasicSceneElement>(staplerName + "SceneElement");
	sceneElement->addComponent(staplerHandle);
	sceneElement->addComponent(staplerIndicator);
	sceneElement->addComponent(staplerMarkings);
	sceneElement->addComponent(staplerTrigger);
	sceneElement->addComponent(glue);
	sceneElement->addComponent(inputComponent);
	sceneElement->addComponent(transferInputPose);
	sceneElement->addComponent(addStapleBehavior);

	return sceneElement;
}

std::shared_ptr<ViewElement> createView()
{
	std::shared_ptr<OsgViewElement> view = std::make_shared<OsgViewElement>("StaplingDemoView");

	view->enableManipulator(true);
	view->setManipulatorParameters(Vector3d(0.0, 0.5, 0.5), Vector3d::Zero());

	return view;
}

int main(int argc, char* argv[])
{
	const std::string deviceName = "MultiAxisDevice";

	std::shared_ptr<BehaviorManager> behaviorManager = std::make_shared<BehaviorManager>();
	std::shared_ptr<OsgManager> graphicsManager = std::make_shared<OsgManager>();
	std::shared_ptr<InputManager> inputManager = std::make_shared<InputManager>();
	std::shared_ptr<PhysicsManager> physicsManager = std::make_shared<PhysicsManager>();

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>("config.txt");
	runtime->addManager(behaviorManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(inputManager);
	runtime->addManager(physicsManager);

	std::shared_ptr<DeviceInterface> toolDevice = std::make_shared<SurgSim::Device::MultiAxisDevice>(deviceName);
	SURGSIM_ASSERT(toolDevice->initialize() == true) <<
		"Could not initialize device " << toolDevice->getName() << " for the tool.\n";
	inputManager->addDevice(toolDevice);

	std::shared_ptr<Scene> scene = runtime->getScene();
	scene->addSceneElement(createSceneryObject("arm", "Geometry/forearm.osgb"));
	scene->addSceneElement(createView());
	scene->addSceneElement(createStapler("stapler", deviceName));

	runtime->execute();

	return 0;
}

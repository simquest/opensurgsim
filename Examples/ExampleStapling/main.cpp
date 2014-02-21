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
#include "SurgSim/Blocks/TransferPoseBehavior.h"
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#include "Examples/ExampleStapling/StaplerBehavior.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/InputManager.h"

#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgCapsuleRepresentation.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"

#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"
#include "SurgSim/Physics/FixedRepresentation.h"

#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/CapsuleShape.h"
#include "SurgSim/Math/RigidTransform.h"


/// Load scenery object from file
/// \param name Name of this scenery representation.
/// \param fileName Name of the file from which the scenery representation is loaded.
/// \return A SceneElement containing the scenery representation.
std::shared_ptr<SurgSim::Framework::SceneElement> createSceneryObject(const std::string& name,
																	  const std::string& fileName,
																	  const SurgSim::Math::RigidTransform3d& pose)
{
	std::shared_ptr<SurgSim::Graphics::SceneryRepresentation> sceneryRepresentation =
		std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>(name + "SceneryRepresentation");
	sceneryRepresentation->setFileName(fileName);
	sceneryRepresentation->setInitialPose(pose);

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

std::shared_ptr<SurgSim::Framework::SceneElement> createStapler(const std::string&name,
																const SurgSim::Math::RigidTransform3d& pose)
{
	// add graphic representation for staplerSceneElement
	std::shared_ptr<SurgSim::Framework::SceneElement> staplerSceneElement =
		createSceneryObject(name, "Geometry/stapler.obj", pose);

	// In order to connect the stapler scenery representation to an input device, after create a SceneElement for it,
	// we need to retrieve the stapler scenery representation from the SceneElement.
	std::shared_ptr<SurgSim::Framework::Component> staplerComponent =
		staplerSceneElement->getComponent("staplerSceneryRepresentation");
	auto staplerSceneryRepresentation = std::static_pointer_cast<SurgSim::Framework::Representation>(staplerComponent);

	// Stapler Physics
	auto shape = std::make_shared<SurgSim::Math::CapsuleShape>(0.172, 0.028);

	SurgSim::Physics::RigidRepresentationParameters params;
	params.setDensity(8050); // Stainless steel
	params.setShapeUsedForMassInertia(shape);

	auto physicsRepresentation = std::make_shared<SurgSim::Physics::RigidRepresentation>(name + "Physics");
	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setInitialPose(pose);

	// Stapler collisionRep
	auto collisionRepresentation =  std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>
		(name + "Collision");
	collisionRepresentation->setRigidRepresentation(physicsRepresentation);

	// Debug only
	// Add the capsule rigid representation
	std::shared_ptr<SurgSim::Graphics::CapsuleRepresentation> capsuleRepresentation =
		std::make_shared<SurgSim::Graphics::OsgCapsuleRepresentation>("capsule representation");
	capsuleRepresentation->setHeight(0.172);
	capsuleRepresentation->setRadius(0.028);
	capsuleRepresentation->setInitialPose(pose);

	auto inputComponent = std::make_shared<SurgSim::Input::InputComponent>("InputComponent");
	inputComponent->setDeviceName("MultiAxisDevice");


	// Behaviors

	// Add VTC
	std::shared_ptr<SurgSim::Physics::VirtualToolCoupler> inputVTC =
		std::make_shared<SurgSim::Physics::VirtualToolCoupler>("VTC");
	inputVTC->setInput(inputComponent);
	inputVTC->setRepresentation(physicsRepresentation);
	inputVTC->setAngularDamping(params.getMass() * 10e-2);
	inputVTC->setAngularStiffness(params.getMass() * 50);
	inputVTC->setLinearDamping(params.getMass() * 10);
	inputVTC->setLinearStiffness(params.getMass() * 200);

	// Add stapler behavior
	auto addStaplerBehavior = std::make_shared<StaplerBehavior>(name + "StaplerBehavior");
	addStaplerBehavior->setInputComponent(inputComponent);
	addStaplerBehavior->setStaplerRepresentation(collisionRepresentation);

	// Transfer physicsRepresentation to sceneryRepresentation raw
	auto transferInputPosePS = std::make_shared<SurgSim::Blocks::TransferInputPoseBehavior>(name + "Input to Physics");
	transferInputPosePS->setPoseSender(inputComponent);
	transferInputPosePS->setPoseReceiver(staplerSceneryRepresentation);

	// Transfer physics to rigidRepresentation shape
	auto transferInputPosePR = std::make_shared<SurgSim::Blocks::TransferPoseBehavior>(name + "Graphic to RigidRep");
	transferInputPosePR->setPoseSender(physicsRepresentation);
	transferInputPosePR->setPoseReceiver(capsuleRepresentation);

	staplerSceneElement->addComponent(physicsRepresentation);
	staplerSceneElement->addComponent(collisionRepresentation);
	staplerSceneElement->addComponent(capsuleRepresentation);

	staplerSceneElement->addComponent(inputComponent);
	staplerSceneElement->addComponent(addStaplerBehavior);
	staplerSceneElement->addComponent(inputVTC);
	staplerSceneElement->addComponent(transferInputPosePR);
	staplerSceneElement->addComponent(transferInputPosePS);

	return staplerSceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createArm(const std::string&name,
															const SurgSim::Math::RigidTransform3d& pose)
{
	// add graphic representation for armSceneElement
	std::shared_ptr<SurgSim::Framework::SceneElement> armSceneElement =
		createSceneryObject(name, "Geometry/forearm.osgb", pose);

	// Debug only
	// Add the capsule representation
	std::shared_ptr<SurgSim::Graphics::BoxRepresentation> boxRepresentation =
		std::make_shared<SurgSim::Graphics::OsgBoxRepresentation>("capsule representation");
	boxRepresentation->setSizeX(0.635);
	boxRepresentation->setSizeY(0.05);
	boxRepresentation->setSizeZ(0.05);
	boxRepresentation->setInitialPose(pose);

	// Arm Physics
	auto shape = std::make_shared<SurgSim::Math::BoxShape>(0.635, 0.05, 0.05);

	SurgSim::Physics::RigidRepresentationParameters params;
	params.setDensity(1062); // Average human body density
	params.setShapeUsedForMassInertia(shape);

	auto physicsRepresentation = std::make_shared<SurgSim::Physics::FixedRepresentation>(name + "Physics");
	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setInitialPose(pose);

	// Arm collisionRep
	auto collisionRepresentation =  std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>
		(name + "Collision");
	collisionRepresentation->setRigidRepresentation(physicsRepresentation);

	// Behavior
	auto transferPose = std::make_shared<SurgSim::Blocks::TransferPoseBehavior>("Physics to Graphics Pose");
	transferPose->setPoseSender(physicsRepresentation);
	transferPose->setPoseReceiver(boxRepresentation);

	armSceneElement->addComponent(physicsRepresentation);
	armSceneElement->addComponent(collisionRepresentation);

	armSceneElement->addComponent(boxRepresentation);
	armSceneElement->addComponent(transferPose);

	return armSceneElement;
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

	// create staplerSceneElement
	std::shared_ptr<SurgSim::Framework::SceneElement> staplerSceneElement = createStapler("stapler", SurgSim::Math::
			makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), SurgSim::Math::Vector3d(0.0, 0.0, 0.0)));

	// create armSceneElement
	std::shared_ptr<SurgSim::Framework::SceneElement> armSceneElement = createArm("arm", SurgSim::Math::
		makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), SurgSim::Math::Vector3d(0.0, -0.2, 0.0)));

	std::shared_ptr<SurgSim::Framework::Scene> scene = runtime->getScene();
	scene->addSceneElement(staplerSceneElement);
	scene->addSceneElement(armSceneElement);

	scene->addSceneElement(createView());

	runtime->execute();

	return 0;
}

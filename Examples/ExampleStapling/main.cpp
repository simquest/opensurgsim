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
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgCapsuleRepresentation.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/CapsuleShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

using SurgSim::Blocks::TransferInputPoseBehavior;
using SurgSim::Blocks::TransferPoseBehavior;
using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Framework::Runtime;
using SurgSim::Device::MultiAxisDevice;
using SurgSim::Graphics::BoxRepresentation;
using SurgSim::Graphics::CapsuleRepresentation;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::OsgCapsuleRepresentation;
using SurgSim::Graphics::OsgManager;
using SurgSim::Graphics::OsgSceneryRepresentation;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::SceneryRepresentation;
using SurgSim::Graphics::ViewElement;
using SurgSim::Math::BoxShape;
using SurgSim::Math::CapsuleShape;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Input::DeviceInterface;
using SurgSim::Input::InputComponent;
using SurgSim::Input::InputManager;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::PhysicsManager;
using SurgSim::Physics::RigidRepresentationParameters;
using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::VirtualToolCoupler;

/// Load scenery object from file.
/// \param name Name of this scenery representation.
/// \param fileName Name of the file from which the scenery representation is loaded.
/// \param pose The pose of the loaded scenery representation.
/// \return A SceneElement containing the scenery representation.
std::shared_ptr<SceneElement> createSceneryObject(const std::string& name,
												  const std::string& fileName,
												  const RigidTransform3d& pose)
{
	std::shared_ptr<SceneryRepresentation> sceneryRepresentation =
		std::make_shared<OsgSceneryRepresentation>(name + "SceneryRepresentation");
	sceneryRepresentation->setFileName(fileName);
	sceneryRepresentation->setInitialPose(pose);

	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>(name + "SceneElement");
	sceneElement->addComponent(sceneryRepresentation);

	return sceneElement;
}

std::shared_ptr<ViewElement> createView()
{
	std::shared_ptr<OsgViewElement> view = std::make_shared<OsgViewElement>("StaplingDemoView");

	view->enableManipulator(true);
	view->setManipulatorParameters(Vector3d(0.0, 0.5, 0.5), Vector3d::Zero());

	return view;
}

std::shared_ptr<SceneElement> createStapler(const std::string& name)
{
	// Since there is no collision mesh loader yet, use a capsule shape as the collision representation of the stapler.
	std::shared_ptr<CapsuleShape> capsuleShape = std::make_shared<CapsuleShape>(0.1, 0.02); // Unit: meter
	RigidRepresentationParameters params;
	params.setDensity(8050); // Stainless steel (in Kg.m-3)
	params.setShapeUsedForMassInertia(capsuleShape);
	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>(name + "Physics");
	physicsRepresentation->setInitialParameters(params);

	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>(name + "Collision");
	collisionRepresentation->setRigidRepresentation(physicsRepresentation);

	std::shared_ptr<InputComponent> inputComponent = std::make_shared<InputComponent>("InputComponent");
	inputComponent->setDeviceName("MultiAxisDevice");

	std::shared_ptr<VirtualToolCoupler> inputVTC = std::make_shared<VirtualToolCoupler>("VTC");
	inputVTC->setInput(inputComponent);
	inputVTC->setRepresentation(physicsRepresentation);
	inputVTC->setAngularDamping(params.getMass() * 10e-2);
	inputVTC->setAngularStiffness(params.getMass() * 50.0);
	inputVTC->setLinearDamping(params.getMass() * 10.0);
	inputVTC->setLinearStiffness(params.getMass() * 200.0);

	// A stapler behavior controls the release of stale when a button is pushed on the device.
	// Also, it is aware of collisions of the stapler.
	std::shared_ptr<StaplerBehavior> staplerBehavior = std::make_shared<StaplerBehavior>(name + "StaplerBehavior");
	staplerBehavior->setInputComponent(inputComponent);
	staplerBehavior->setStaplerRepresentation(collisionRepresentation);

	std::shared_ptr<SceneryRepresentation> staplerSceneryRepresentation =
		std::make_shared<OsgSceneryRepresentation>(name + "SceneryRepresentation");
	staplerSceneryRepresentation->setFileName("Geometry/stapler.obj");
	// Connect inputComponent to graphical representation.
	std::shared_ptr<TransferInputPoseBehavior> transferInputPose =
		std::make_shared<TransferInputPoseBehavior>(name + "Input to Graphics");
	transferInputPose->setPoseSender(inputComponent);
	transferInputPose->setPoseReceiver(staplerSceneryRepresentation);

	// Visualization of the collision representation.
	std::shared_ptr<CapsuleRepresentation> osgCapsuleRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("capsule representation");
	osgCapsuleRepresentation->setHeight(capsuleShape->getLength());
	osgCapsuleRepresentation->setRadius(capsuleShape->getRadius());
	// Connect physical representation to graphical representation.
	std::shared_ptr<TransferPoseBehavior> transferPose =
		std::make_shared<TransferPoseBehavior>(name + "Physics to Graphics");
	transferPose->setPoseSender(physicsRepresentation);
	transferPose->setPoseReceiver(osgCapsuleRepresentation);

	std::shared_ptr<SceneElement> staplerSceneElement = std::make_shared<BasicSceneElement>(name + "SceneElement");
	staplerSceneElement->addComponent(physicsRepresentation);
	staplerSceneElement->addComponent(collisionRepresentation);
	staplerSceneElement->addComponent(osgCapsuleRepresentation);
	staplerSceneElement->addComponent(staplerSceneryRepresentation);
	staplerSceneElement->addComponent(inputComponent);
	staplerSceneElement->addComponent(inputVTC);
	staplerSceneElement->addComponent(staplerBehavior);
	staplerSceneElement->addComponent(transferInputPose);
	staplerSceneElement->addComponent(transferPose);

	return staplerSceneElement;
}

std::shared_ptr<SceneElement> createArm(const std::string& name, const RigidTransform3d& pose)
{
	// Load graphic representation for armSceneElement
	std::shared_ptr<SceneElement> armSceneElement = createSceneryObject(name, "Geometry/forearm.osgb", pose);

	std::shared_ptr<BoxRepresentation> boxRepresentation =
		std::make_shared<OsgBoxRepresentation>("capsule representation");
	boxRepresentation->setSize(0.635, 0.05, 0.05); // Unit: meter
	boxRepresentation->setInitialPose(pose);

	// Since there is no collision mesh loader yet, use a box shape as the collision representation of the arm.
	std::shared_ptr<BoxShape> boxShape = std::make_shared<BoxShape>(boxRepresentation->getSizeX(),
																	boxRepresentation->getSizeY(),
																	boxRepresentation->getSizeZ());

	RigidRepresentationParameters params;
	params.setDensity(1062); // Average human body density  (in Kg.m-3)
	params.setShapeUsedForMassInertia(boxShape);

	std::shared_ptr<FixedRepresentation> physicsRepresentation =
		std::make_shared<FixedRepresentation>(name + "Physics");
	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setInitialPose(pose);

	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>(name + "Collision");
	collisionRepresentation->setRigidRepresentation(physicsRepresentation);

	std::shared_ptr<TransferPoseBehavior> transferPose =
		std::make_shared<TransferPoseBehavior>("Physics to Graphics Pose");
	transferPose->setPoseSender(physicsRepresentation);
	transferPose->setPoseReceiver(boxRepresentation);

	armSceneElement->addComponent(boxRepresentation);
	armSceneElement->addComponent(physicsRepresentation);
	armSceneElement->addComponent(collisionRepresentation);
	armSceneElement->addComponent(transferPose);

	return armSceneElement;
}

int main(int argc, char* argv[])
{
	std::shared_ptr<BehaviorManager> behaviorManager = std::make_shared<BehaviorManager>();
	std::shared_ptr<OsgManager> graphicsManager = std::make_shared<OsgManager>();
	std::shared_ptr<InputManager> inputManager    = std::make_shared<InputManager>();
	std::shared_ptr<PhysicsManager>  physicsManager  = std::make_shared<PhysicsManager>();

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>("config.txt");
	runtime->addManager(behaviorManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(inputManager);
	runtime->addManager(physicsManager);

	std::shared_ptr<DeviceInterface> device = std::make_shared<MultiAxisDevice>("MultiAxisDevice");
	SURGSIM_ASSERT(device->initialize() == true) <<
		"Could not initialize device " << device->getName() << " for the tool.\n";
	inputManager->addDevice(device);

	std::shared_ptr<SceneElement> staplerSceneElement = createStapler("stapler");
	std::shared_ptr<SceneElement> armSceneElement = createArm("arm",
		makeRigidTransform(Quaterniond::Identity(), SurgSim::Math::Vector3d(0.0, -0.2, 0.0)));

	std::shared_ptr<Scene> scene = runtime->getScene();
	scene->addSceneElement(createView());
	scene->addSceneElement(staplerSceneElement);
	scene->addSceneElement(armSceneElement);

	runtime->execute();

	return 0;
}

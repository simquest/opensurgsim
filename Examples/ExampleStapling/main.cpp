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

#include <list>
#include <memory>
#include <string>

#include "SurgSim/Blocks/TransferPoseBehavior.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#include "Examples/ExampleStapling/StaplerBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

using SurgSim::Blocks::TransferPoseBehavior;
using SurgSim::DataStructures::PlyReader;
using SurgSim::DataStructures::TriangleMeshPlyReaderDelegate;
using SurgSim::Device::MultiAxisDevice;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::SceneryRepresentation;
using SurgSim::Graphics::ViewElement;
using SurgSim::Graphics::OsgManager;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::OsgSceneryRepresentation;
using SurgSim::Graphics::ViewElement;
using SurgSim::Math::MeshShape;
using SurgSim::Math::MeshShape;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationMatrix;
using SurgSim::Math::Matrix33d;
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

/// Load scenery object from file
/// \param name Name of this scenery representation.
/// \param fileName Name of the file from which the scenery representation will be loaded.
/// \return A scenery representation.
std::shared_ptr<SceneryRepresentation> createSceneryObject(const std::string& name, const std::string& fileName)
{
	std::shared_ptr<SceneryRepresentation> sceneryRepresentation = std::make_shared<OsgSceneryRepresentation>(name);
	sceneryRepresentation->setFileName(fileName);
	return sceneryRepresentation;
}

std::shared_ptr<ViewElement> createView()
{
	std::shared_ptr<OsgViewElement> view = std::make_shared<OsgViewElement>("StaplingDemoView");

	view->enableManipulator(true);
	view->setManipulatorParameters(Vector3d(0.0, 0.5, 0.5), Vector3d::Zero());

	return view;
}

std::shared_ptr<SceneElement> createStapler(const std::string& staplerName, const std::string& deviceName)
{
	std::shared_ptr<TriangleMeshPlyReaderDelegate> delegate = std::make_shared<TriangleMeshPlyReaderDelegate>();
	PlyReader reader("Data/Geometry/stapler_collision.ply");
	reader.setDelegate(delegate);
	reader.parseFile();

	// Stapler collision mesh
	std::shared_ptr<MeshShape> meshShape = std::make_shared<MeshShape>(*delegate->getMesh()); // Unit: meter
	RigidRepresentationParameters params;
	params.setDensity(8050); // Stainless steel (in Kg.m-3)
	params.setShapeUsedForMassInertia(meshShape);

	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>(staplerName + "Physics");
	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setIsGravityEnabled(false);

	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>(staplerName + "Collision");
	collisionRepresentation->setRigidRepresentation(physicsRepresentation);

	std::shared_ptr<InputComponent> inputComponent = std::make_shared<InputComponent>("InputComponent");
	inputComponent->setDeviceName(deviceName);

	std::shared_ptr<VirtualToolCoupler> inputVTC = std::make_shared<VirtualToolCoupler>("VTC");
	inputVTC->setInput(inputComponent);
	inputVTC->setRepresentation(physicsRepresentation);
	inputVTC->setAngularDamping(params.getMass() * 25e-2);
	inputVTC->setAngularStiffness(params.getMass() * 10.0);
	inputVTC->setLinearDamping(params.getMass() * 25);
	inputVTC->setLinearStiffness(params.getMass() * 800.0);

	// A stapler behavior controls the release of stale when a button is pushed on the device.
	// Also, it is aware of collisions of the stapler.
	std::shared_ptr<StaplerBehavior> staplerBehavior = std::make_shared<StaplerBehavior>(staplerName + "Behavior");
	staplerBehavior->setInputComponent(inputComponent);
	staplerBehavior->setCollisionRepresentation(collisionRepresentation);

	std::shared_ptr<SceneElement> sceneElement = std::make_shared<BasicSceneElement>(staplerName + "SceneElement");
	sceneElement->addComponent(physicsRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(inputComponent);
	sceneElement->addComponent(inputVTC);
	sceneElement->addComponent(staplerBehavior);

	// Load the graphical parts of a stapler.
	std::list<std::shared_ptr<SceneryRepresentation>> sceneryRepresentations;
	sceneryRepresentations.push_back(createSceneryObject(staplerName + "Handle",    "Geometry/stapler_handle.obj"));
	sceneryRepresentations.push_back(createSceneryObject(staplerName + "Indicator", "Geometry/stapler_indicator.obj"));
	sceneryRepresentations.push_back(createSceneryObject(staplerName + "Markings",  "Geometry/stapler_markings.obj"));
	sceneryRepresentations.push_back(createSceneryObject(staplerName + "Trigger",   "Geometry/stapler_trigger.obj"));
	for (auto it = std::begin(sceneryRepresentations); it != std::end(sceneryRepresentations); ++it)
	{
		std::shared_ptr<TransferPoseBehavior> transferPhysicsPoseToGraphics =
			std::make_shared<TransferPoseBehavior>("Physics to Graphics" + (*it)->getName());
		transferPhysicsPoseToGraphics->setPoseSender(physicsRepresentation);
		transferPhysicsPoseToGraphics->setPoseReceiver(*it);

		sceneElement->addComponent(*it);
		sceneElement->addComponent(transferPhysicsPoseToGraphics);
	}

	return sceneElement;
}

std::shared_ptr<SceneElement> createArm(const std::string& armName, const RigidTransform3d& pose)
{
	std::shared_ptr<TriangleMeshPlyReaderDelegate> delegate = std::make_shared<TriangleMeshPlyReaderDelegate>();
	PlyReader reader("Data/Geometry/arm_collision.ply");
	reader.setDelegate(delegate);
	reader.parseFile();

	// Load graphic representation for armSceneElement
	std::shared_ptr<SceneryRepresentation> sceneryRepresentation =
		createSceneryObject(armName, "Geometry/forearm.osgb");
	sceneryRepresentation->setInitialPose(pose);

	// MeshShape collision representation of the arm.
	std::shared_ptr<MeshShape> meshShape = std::make_shared<MeshShape>(*delegate->getMesh()); // Unit: meter
	RigidRepresentationParameters params;
	params.setDensity(1062); // Average human body density  (in Kg.m-3)
	params.setShapeUsedForMassInertia(meshShape);

	Matrix33d rotationX = makeRotationMatrix(M_PI_2, Vector3d(1.0, 0.0, 0.0));
	Matrix33d rotationY = makeRotationMatrix(M_PI_4, Vector3d(0.0, 1.0, 0.0));
	RigidTransform3d alignedPose = makeRigidTransform(pose.linear() * rotationY * rotationX, pose.translation());

	std::shared_ptr<FixedRepresentation> physicsRepresentation =
		std::make_shared<FixedRepresentation>(armName + "Physics");
	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setInitialPose(alignedPose);

	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>(armName + "Collision");
	collisionRepresentation->setRigidRepresentation(physicsRepresentation);

	std::shared_ptr<SceneElement> armSceneElement = std::make_shared<BasicSceneElement>("ArmSceneElement");
	armSceneElement->addComponent(sceneryRepresentation);
	armSceneElement->addComponent(collisionRepresentation);
	armSceneElement->addComponent(physicsRepresentation);

	return armSceneElement;
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

	std::shared_ptr<DeviceInterface> device = std::make_shared<MultiAxisDevice>(deviceName);
	SURGSIM_ASSERT(device->initialize() == true) <<
			"Could not initialize device " << device->getName() << " for the tool.\n";
	inputManager->addDevice(device);

	std::shared_ptr<Scene> scene = runtime->getScene();
	scene->addSceneElement(createView());
	scene->addSceneElement(createArm("arm", makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, -0.2, 0.0))));
	scene->addSceneElement(createStapler("stapler", deviceName));

	runtime->execute();

	return 0;
}

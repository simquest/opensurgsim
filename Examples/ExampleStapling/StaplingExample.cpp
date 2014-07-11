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

#include "Examples/ExampleStapling/StaplerBehavior.h"
#include "SurgSim/Blocks/KeyboardTogglesGraphicsBehavior.h"
#include "SurgSim/Blocks/TransferPhysicsToGraphicsMeshBehavior.h"
#include "SurgSim/Blocks/VisualizeContactsBehavior.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

using SurgSim::Blocks::KeyboardTogglesGraphicsBehavior;
using SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior;
using SurgSim::Blocks::VisualizeContactsBehavior;
using SurgSim::Collision::ShapeCollisionRepresentation;
using SurgSim::Device::IdentityPoseDevice;
using SurgSim::Device::MultiAxisDevice;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::MeshRepresentation;
using SurgSim::Graphics::SceneryRepresentation;
using SurgSim::Graphics::OsgManager;
using SurgSim::Graphics::OsgMeshRepresentation;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::OsgSceneryRepresentation;
using SurgSim::Math::MeshShape;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationMatrix;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Input::DeviceInterface;
using SurgSim::Input::InputComponent;
using SurgSim::Input::InputManager;
using SurgSim::Physics::DeformableCollisionRepresentation;
using SurgSim::Physics::Fem3DRepresentation;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::PhysicsManager;
using SurgSim::Physics::RigidRepresentationParameters;
using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::VirtualToolCoupler;

static std::shared_ptr<SurgSim::Framework::SceneElement> createFemSceneElement(
	const std::string& name,
	const std::string& filename,
	SurgSim::Math::IntegrationScheme integrationScheme)
{
	// Create a SceneElement that bundles the pieces associated with the finite element model
	std::shared_ptr<SceneElement> sceneElement = std::make_shared<BasicSceneElement>(name);

	// Set the file name which contains the tetrahedral mesh. File will be loaded by 'doInitialize()' call.
	std::shared_ptr<Fem3DRepresentation> physicsRepresentation = std::make_shared<Fem3DRepresentation>("Physics");
	physicsRepresentation->setFilename(filename);
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	sceneElement->addComponent(physicsRepresentation);

	// Load the surface triangle mesh of the finite element model
	auto meshShape = std::make_shared<MeshShape>();
	meshShape->setFileName(filename);

	// Create a triangle mesh for visualizing the surface of the finite element model
	std::shared_ptr<SurgSim::Graphics::MeshRepresentation> graphicalFem =
		std::make_shared<OsgMeshRepresentation>("Triangle mesh");
	graphicalFem->setFilename(filename);
	sceneElement->addComponent(graphicalFem);

	// Create the collision mesh for the surface of the finite element model
	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision");
	collisionRepresentation->setShape(meshShape);
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);
	sceneElement->addComponent(collisionRepresentation);

	// Create a behavior which transfers the position of the vertices in the FEM to locations in the triangle mesh
	auto physicsToGraphicalFem = std::make_shared<TransferPhysicsToGraphicsMeshBehavior>("PhysicsToGraphicalFem");
	physicsToGraphicalFem->setSource(physicsRepresentation);
	physicsToGraphicalFem->setTarget(graphicalFem);
	sceneElement->addComponent(physicsToGraphicalFem);

	// WireFrame of the finite element model
	std::shared_ptr<SurgSim::Graphics::MeshRepresentation> wireFrameFem
		= std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("Wire frame");
	wireFrameFem->setFilename(filename);
	wireFrameFem->setDrawAsWireFrame(true);
	sceneElement->addComponent(wireFrameFem);

	// Behavior transfers the position of the physics representation to wire frame representation of the fem.
	auto physicsToWireFrameFem = std::make_shared<TransferPhysicsToGraphicsMeshBehavior>("PhysicsToWireFrameFem");
	physicsToWireFrameFem->setSource(physicsRepresentation);
	physicsToWireFrameFem->setTarget(wireFrameFem);
	sceneElement->addComponent(physicsToWireFrameFem);

	return sceneElement;
}

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

std::shared_ptr<SceneElement> createStaplerSceneElement(const std::string& staplerName, const std::string& deviceName)
{
	const std::string filename = std::string("Geometry/stapler_collision.ply");

	// Stapler collision mesh
	auto meshShapeForCollision = std::make_shared<MeshShape>();
	meshShapeForCollision->setFileName(filename);

	std::shared_ptr<MeshRepresentation> meshShapeVisualization =
		std::make_shared<OsgMeshRepresentation>("StaplerOsgMesh");
	meshShapeVisualization->setFilename(filename);
	meshShapeVisualization->setDrawAsWireFrame(true);

	RigidRepresentationParameters params;
	params.setDensity(8050); // Stainless steel (in Kg.m-3)
	params.setShapeUsedForMassInertia(meshShapeForCollision);

	std::shared_ptr<RigidRepresentation> physicsRepresentation = std::make_shared<RigidRepresentation>("Physics");
	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setIsGravityEnabled(false);

	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	std::shared_ptr<InputComponent> inputComponent = std::make_shared<InputComponent>("InputComponent");
	inputComponent->setDeviceName(deviceName);

	std::shared_ptr<VirtualToolCoupler> inputVTC = std::make_shared<VirtualToolCoupler>("VTC");
	inputVTC->setInput(inputComponent);
	inputVTC->setRepresentation(physicsRepresentation);

	// A stapler behavior controls the release of stale when a button is pushed on the device.
	// Also, it is aware of collisions of the stapler.
	std::shared_ptr<StaplerBehavior> staplerBehavior = std::make_shared<StaplerBehavior>("Behavior");
	staplerBehavior->setInputComponent(inputComponent);
	staplerBehavior->setRepresentation(physicsRepresentation);
	staplerBehavior->enableStaplingForSceneElement("armSceneElement");

	std::shared_ptr<VisualizeContactsBehavior> visualizeContactsBehavior =
		std::make_shared<VisualizeContactsBehavior>("VisualizeContactsBehavior");
	visualizeContactsBehavior->setCollisionRepresentation(collisionRepresentation);
	// Note: Since usually the penetration depth of a collision is so small (at the magnitude of mm),
	// if we use the depth as the length of vector, the vector field will be too small to be seen on the screen.
	// Thus, we enlarge the vector field by 200 times.
	visualizeContactsBehavior->setVectorFieldScale(200);

	std::shared_ptr<SceneElement> sceneElement = std::make_shared<BasicSceneElement>(staplerName + "SceneElement");
	sceneElement->addComponent(physicsRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(meshShapeVisualization);
	sceneElement->addComponent(inputComponent);
	sceneElement->addComponent(inputVTC);
	sceneElement->addComponent(staplerBehavior);
	sceneElement->addComponent(visualizeContactsBehavior);

	// Load the graphical parts of a stapler.
	sceneElement->addComponent(createSceneryObject("Handle",    "Geometry/stapler_handle.obj"));
	sceneElement->addComponent(createSceneryObject("Indicator", "Geometry/stapler_indicator.obj"));
	sceneElement->addComponent(createSceneryObject("Markings",  "Geometry/stapler_markings.obj"));
	sceneElement->addComponent(createSceneryObject("Trigger",   "Geometry/stapler_trigger.obj"));

	auto meshShapeForVirtualStaple1 = std::make_shared<MeshShape>();
	auto meshShapeForVirtualStaple2 = std::make_shared<MeshShape>();
	meshShapeForVirtualStaple1->setFileName("Geometry/virtual_staple_1.ply");
	meshShapeForVirtualStaple2->setFileName("Geometry/virtual_staple_2.ply");

	std::vector<std::shared_ptr<MeshShape>> virtualTeethShapes;
	virtualTeethShapes.push_back(meshShapeForVirtualStaple1);
	virtualTeethShapes.push_back(meshShapeForVirtualStaple2);

	int i = 0;
	std::array<std::shared_ptr<SurgSim::Collision::Representation>, 2> virtualTeeth;
	for (auto it = std::begin(virtualTeethShapes); it != std::end(virtualTeethShapes); ++it, ++i)
	{
		std::shared_ptr<ShapeCollisionRepresentation> virtualToothCollision =
			std::make_shared<ShapeCollisionRepresentation>("VirtualToothCollision" + std::to_string(i));
		virtualToothCollision->setShape(*it);
		virtualToothCollision->setLocalPose(RigidTransform3d::Identity());

		virtualTeeth[i] = virtualToothCollision;
		sceneElement->addComponent(virtualToothCollision);

		std::shared_ptr<MeshRepresentation> virtualToothMesh
			= std::make_shared<OsgMeshRepresentation>("virtualToothMesh" + std::to_string(i));
		virtualToothMesh->setFilename((*it)->getFileName());
		virtualToothMesh->setDrawAsWireFrame(true);

		sceneElement->addComponent(virtualToothMesh);
	}

	staplerBehavior->setVirtualTeeth(virtualTeeth);

	return sceneElement;
}

std::shared_ptr<SceneElement> createArmSceneElement(const std::string& armName)
{
	const std::string filename = std::string("Geometry/arm_collision.ply");

	// Graphic representation for arm
	std::shared_ptr<SceneryRepresentation> forearmSceneryRepresentation =
		createSceneryObject("forearm", "Geometry/forearm.osgb");
	std::shared_ptr<SceneryRepresentation> upperarmSceneryRepresentation =
		createSceneryObject("upperarm", "Geometry/upperarm.osgb");

	// Arm collision mesh
	std::shared_ptr<MeshShape> meshShape = std::make_shared<MeshShape>();
	meshShape->setFileName(filename);

	// Visualization of arm collision mesh
	std::shared_ptr<MeshRepresentation> meshShapeVisualization = std::make_shared<OsgMeshRepresentation>("ArmOsgMesh");
	meshShapeVisualization->setFilename(filename);
	meshShapeVisualization->setDrawAsWireFrame(true);

	RigidRepresentationParameters params;
	params.setShapeUsedForMassInertia(meshShape);

	std::shared_ptr<FixedRepresentation> physicsRepresentation = std::make_shared<FixedRepresentation>("Physics");
	physicsRepresentation->setInitialParameters(params);

	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	std::shared_ptr<SceneElement> armSceneElement = std::make_shared<BasicSceneElement>(armName + "SceneElement");
	armSceneElement->addComponent(forearmSceneryRepresentation);
	armSceneElement->addComponent(meshShapeVisualization);
	armSceneElement->addComponent(upperarmSceneryRepresentation);
	armSceneElement->addComponent(collisionRepresentation);
	armSceneElement->addComponent(physicsRepresentation);

	return armSceneElement;
}

template <typename Type>
std::shared_ptr<Type> getComponentChecked(std::shared_ptr<SurgSim::Framework::SceneElement> sceneElement,
										  const std::string& name)
{
	std::shared_ptr<SurgSim::Framework::Component> component = sceneElement->getComponent(name);
	SURGSIM_ASSERT(component != nullptr) << "Failed to get Component named '" << name << "'.";

	std::shared_ptr<Type> result = std::dynamic_pointer_cast<Type>(component);
	SURGSIM_ASSERT(result != nullptr) << "Failed to convert Component to requested type.";

	return result;
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

	std::shared_ptr<DeviceInterface> device;
	device = std::make_shared<MultiAxisDevice>(deviceName);
	if (!device->initialize())
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Could not initialize device " << device->getName() << " for the tool.";

		device = std::make_shared<IdentityPoseDevice>(deviceName);
	}
	inputManager->addDevice(device);

	std::shared_ptr<OsgViewElement> view = std::make_shared<OsgViewElement>("StaplingDemoView");
	view->enableManipulator(true);
	view->setManipulatorParameters(Vector3d(0.0, 0.5, 0.5), Vector3d::Zero());
	view->enableKeyboardDevice(true);
	inputManager->addDevice(view->getKeyboardDevice());

	RigidTransform3d armPose = makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, -0.2, 0.0));

	std::shared_ptr<SceneElement> arm = createArmSceneElement("arm");
	arm->setPose(armPose);

	std::shared_ptr<SceneElement> stapler = createStaplerSceneElement("stapler", deviceName);
	stapler->setPose(RigidTransform3d::Identity());

	std::string woundFilename = std::string("Geometry/wound_deformable.ply");
	// Mechanical properties are based on Liang and Boppart, "Biomechanical Properties of In Vivo Human Skin From
	// Dynamic Optical Coherence Elastography", IEEE Transactions on Biomedical Engineering, Vol 57, No 4.
	std::shared_ptr<SceneElement> wound =
		createFemSceneElement("wound",
							  woundFilename,
							  SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER);
	wound->setPose(armPose);

	std::shared_ptr<InputComponent> keyboardComponent = std::make_shared<InputComponent>("KeyboardInputComponent");
	keyboardComponent->setDeviceName("Keyboard"); // Name of device is case sensitive.
	std::shared_ptr<KeyboardTogglesGraphicsBehavior> keyboardBehavior =
		std::make_shared<KeyboardTogglesGraphicsBehavior>("KeyboardBehavior");
	keyboardBehavior->setInputComponent(keyboardComponent);

	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_A, stapler->getComponent("Handle"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_A, stapler->getComponent("Indicator"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_A, stapler->getComponent("Markings"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_A, stapler->getComponent("Trigger"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_B, stapler->getComponent("StaplerOsgMesh"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_C, arm->getComponent("forearm"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_C, arm->getComponent("upperarm"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_D, arm->getComponent("ArmOsgMesh"));
	keyboardBehavior->registerKey(
		SurgSim::Device::KeyCode::KEY_E, wound->getComponent("Triangle mesh"));
	keyboardBehavior->registerKey(
		SurgSim::Device::KeyCode::KEY_F, wound->getComponent("Wire frame"));

	std::shared_ptr<SceneElement> keyboard = std::make_shared<BasicSceneElement>("SceneElement");
	keyboard->addComponent(keyboardComponent);
	keyboard->addComponent(keyboardBehavior);

	std::shared_ptr<Scene> scene = runtime->getScene();
	scene->addSceneElement(view);
	scene->addSceneElement(arm);
	scene->addSceneElement(stapler);
	scene->addSceneElement(wound);
	scene->addSceneElement(keyboard);

	// Exclude collision between certain Collision::Representations
	physicsManager->addExcludedCollisionPair(
		getComponentChecked<SurgSim::Collision::Representation>(stapler, "Collision"),
		getComponentChecked<SurgSim::Collision::Representation>(stapler, "VirtualToothCollision0"));

	physicsManager->addExcludedCollisionPair(
		getComponentChecked<SurgSim::Collision::Representation>(stapler, "Collision"),
		getComponentChecked<SurgSim::Collision::Representation>(stapler, "VirtualToothCollision1"));

	physicsManager->addExcludedCollisionPair(
		getComponentChecked<SurgSim::Collision::Representation>(wound, "Collision"),
		getComponentChecked<SurgSim::Collision::Representation>(arm, "Collision"));

	runtime->execute();
	return 0;
}

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
#include <atomic>

#include "Examples/Stapling/DeviceFactory.h"
#include "Examples/Stapling/StaplerBehavior.h"
#include "Examples/Stapling/StaplingPhysicsManager.h"
#include "Examples/Stapling/KeyToQuitBehavior.h"
#include "SurgSim/Blocks/Blocks.h"
#include "SurgSim/Collision/Collision.h"
#include "SurgSim/DataStructures/DataStructures.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Input/Input.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Physics/Physics.h"

#include "Examples/Stapling/StaplerBehavior.h"
#include "Examples/Stapling/Graphics.h"

using SurgSim::Blocks::KeyboardTogglesComponentBehavior;
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
using SurgSim::Input::DeviceInterface;
using SurgSim::Input::InputComponent;
using SurgSim::Input::InputManager;
using SurgSim::Input::OutputComponent;
using SurgSim::Math::MeshShape;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationMatrix;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Vector4f;
using SurgSim::Physics::DeformableCollisionRepresentation;
using SurgSim::Physics::Fem3DRepresentation;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::PhysicsManager;
using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::VirtualToolCoupler;

namespace
{
std::atomic<bool> keepRunning = true;

void stopRunning(int)
{
	keepRunning = false;
}

void run(std::shared_ptr<SurgSim::Framework::Runtime> runtime)
{
	runtime->start();
	while (keepRunning)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	}
	runtime->stop();
}

}

static std::shared_ptr<SurgSim::Framework::SceneElement> createFemSceneElement(
	const std::string& name,
	const std::string& filename,
	SurgSim::Math::IntegrationScheme integrationScheme,
	std::shared_ptr<SurgSim::Graphics::OsgMaterial> material)
{
	// Create a SceneElement that bundles the pieces associated with the finite element model
	std::shared_ptr<SceneElement> sceneElement = std::make_shared<BasicSceneElement>(name);

	// Set the file name which contains the tetrahedral mesh. File will be loaded by 'doInitialize()' call.
	std::shared_ptr<Fem3DRepresentation> physicsRepresentation = std::make_shared<Fem3DRepresentation>("Physics");
	physicsRepresentation->setFilename(filename);
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(0.2);
	physicsRepresentation->setRayleighDampingStiffness(0.2);
	sceneElement->addComponent(physicsRepresentation);

	// Load the surface triangle mesh of the finite element model
	auto meshShape = std::make_shared<MeshShape>();
	meshShape->load(filename);

	// Create a triangle mesh for visualizing the surface of the finite element model
	auto graphicalFem = std::make_shared<OsgMeshRepresentation>("Graphics");
	graphicalFem->loadMesh(filename);
	graphicalFem->addGroupReference("shadowed");
	sceneElement->addComponent(graphicalFem);

	// Create material to transport the Textures
	graphicalFem->setMaterial(material);
	graphicalFem->setGenerateTangents(true);
	sceneElement->addComponent(material);

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
		= std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("Wire Frame");
	wireFrameFem->setShape(meshShape);
	wireFrameFem->setDrawAsWireFrame(true);
	wireFrameFem->setLocalActive(false);
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
std::shared_ptr<OsgSceneryRepresentation> createSceneryObject(const std::string& name, const std::string& fileName)
{
	auto sceneryRepresentation = std::make_shared<OsgSceneryRepresentation>(name);
	sceneryRepresentation->loadModel(fileName);
	return sceneryRepresentation;
}

std::shared_ptr<SceneElement> createStaplerSceneElement(const std::string& staplerName, const std::string& deviceName)
{
	const std::string filename = std::string("Tools/Stapler/stapler_collision.ply");

	// Stapler collision mesh
	auto meshShapeForCollision = std::make_shared<MeshShape>();
	meshShapeForCollision->load(filename);

	std::shared_ptr<MeshRepresentation> meshShapeVisualization =
		std::make_shared<OsgMeshRepresentation>("Collision Mesh");
	meshShapeVisualization->setShape(meshShapeForCollision);
	meshShapeVisualization->setDrawAsWireFrame(true);
	meshShapeVisualization->setLocalActive(false);

	std::shared_ptr<RigidRepresentation> physicsRepresentation = std::make_shared<RigidRepresentation>("Physics");
	physicsRepresentation->setIsGravityEnabled(false);
	physicsRepresentation->setDensity(8050); // Stainless steel (in Kg.m-3)
	physicsRepresentation->setShape(meshShapeForCollision);

	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	std::shared_ptr<InputComponent> inputComponent = std::make_shared<InputComponent>("InputComponent");
	inputComponent->setDeviceName(deviceName);
	auto deviceTransform = SurgSim::Math::makeRigidTranslation(SurgSim::Math::Vector3d(0.0, 0.01, 0.0));
	inputComponent->setLocalPose(deviceTransform);

	std::shared_ptr<OutputComponent> outputComponent = std::make_shared<OutputComponent>("OutputComponent");
	outputComponent->setDeviceName(deviceName);
	outputComponent->setLocalPose(deviceTransform);

	std::shared_ptr<VirtualToolCoupler> inputVTC = std::make_shared<VirtualToolCoupler>("VTC");
	inputVTC->setInput(inputComponent);
	inputVTC->setOutput(outputComponent);
	inputVTC->setRepresentation(physicsRepresentation);
	inputVTC->overrideAttachmentPoint(Vector3d::Zero());
	inputVTC->setCalculateInertialTorques(false);
	inputVTC->overrideAngularStiffness(1.0);
	inputVTC->overrideAngularDamping(0.13);
	inputVTC->overrideLinearStiffness(100.0);
	inputVTC->setHapticOutputOnlyWhenColliding(true);

	// A stapler behavior controls the release of stale when a button is pushed on the device.
	// Also, it is aware of collisions of the stapler.
	std::shared_ptr<StaplerBehavior> staplerBehavior = std::make_shared<StaplerBehavior>("Behavior");
	staplerBehavior->setInputComponent(inputComponent);
	staplerBehavior->setRepresentation(physicsRepresentation);
	staplerBehavior->enableStaplingForSceneElement("wound");

	std::shared_ptr<VisualizeContactsBehavior> visualizeContactsBehavior =
		std::make_shared<VisualizeContactsBehavior>("Contacts");
	visualizeContactsBehavior->setCollisionRepresentation(collisionRepresentation);
	// Note: Since usually the penetration depth of a collision is so small (at the magnitude of mm),
	// if we use the depth as the length of vector, the vector field will be too small to be seen on the screen.
	// Thus, we enlarge the vector field by 200 times.
	visualizeContactsBehavior->setVectorFieldScale(200);
	visualizeContactsBehavior->setLocalActive(false);

	std::shared_ptr<SceneElement> sceneElement = std::make_shared<BasicSceneElement>(staplerName);
	sceneElement->addComponent(physicsRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(meshShapeVisualization);
	sceneElement->addComponent(inputComponent);
	sceneElement->addComponent(outputComponent);
	sceneElement->addComponent(inputVTC);
	sceneElement->addComponent(staplerBehavior);
	sceneElement->addComponent(visualizeContactsBehavior);

	// Load the graphical parts of a stapler.
	auto component = createSceneryObject("Stapler", "Tools/Stapler/stapler.osg");
	component->addGroupReference("shadowed");
	component->addGroupReference("shadowing");
	sceneElement->addComponent(component);

	component = createSceneryObject("Handle", "Tools/Stapler/handle.osg");
	component->addGroupReference("shadowed");
	component->addGroupReference("shadowing");
	sceneElement->addComponent(component);

	component = createSceneryObject("Footplate", "Tools/Stapler/footplate.osg");
	component->addGroupReference("shadowed");
	component->addGroupReference("shadowing");
	sceneElement->addComponent(component);

	auto meshShapeForVirtualStaple1 = std::make_shared<MeshShape>();
	auto meshShapeForVirtualStaple2 = std::make_shared<MeshShape>();
	meshShapeForVirtualStaple1->load("Tools/Stapler/virtual_staple_1.ply");
	meshShapeForVirtualStaple2->load("Tools/Stapler/virtual_staple_2.ply");

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

		std::shared_ptr<MeshRepresentation> toothShapeVisualization =
			std::make_shared<OsgMeshRepresentation>("Tooth Graphics" + boost::to_string(i));
		toothShapeVisualization->setShape(*it);
		toothShapeVisualization->setDrawAsWireFrame(true);
		toothShapeVisualization->setLocalActive(false);
		sceneElement->addComponent(toothShapeVisualization);
	}

	staplerBehavior->setVirtualTeeth(virtualTeeth);

	return sceneElement;
}

std::shared_ptr<SceneElement> createArmSceneElement(const std::string& armName)
{
	auto element = std::make_shared<BasicSceneElement>(armName);
	const std::string filename = "Geometry/arm_collision.ply";

	// Graphic representation for arm
	auto material = SurgSim::Blocks::createNormalMappedMaterial("forearm material",
					Vector4f(1.0, 1.0, 1.0, 1.0),
					Vector4f(0.4, 0.4, 0.4, 1.0), 10.0,
					"",
					"Geometry/forearm_normal.png");
	auto forearm = createSceneryObject("Forearm", "Geometry/forearm.osgb");
	forearm->setMaterial(material);
	forearm->addGroupReference("shadowing");
	forearm->addGroupReference("shadowed");
	forearm->setGenerateTangents(true);
	element->addComponent(forearm);
	element->addComponent(material);

	material = SurgSim::Blocks::createNormalMappedMaterial("upperarm material",
			   Vector4f(1.0, 1.0, 1.0, 1.0),
			   Vector4f(0.4, 0.4, 0.4, 1.0), 10.0,
			   "",
			   "Geometry/upperarm_normal.png");
	auto upperArm = createSceneryObject("Upperarm", "Geometry/upperarm.osgb");
	upperArm->setMaterial(material);
	upperArm->setGenerateTangents(true);
	upperArm->addGroupReference("shadowing");
	upperArm->addGroupReference("shadowed");
	element->addComponent(upperArm);
	element->addComponent(material);

	// Arm collision mesh
	std::shared_ptr<MeshShape> meshShape = std::make_shared<MeshShape>();
	meshShape->load(filename);

	// Visualization of arm collision mesh
	auto meshShapeVisualization = std::make_shared<OsgMeshRepresentation>("Collision Mesh");
	meshShapeVisualization->setShape(meshShape);
	meshShapeVisualization->setDrawAsWireFrame(true);
	meshShapeVisualization->setLocalActive(false);

	auto physicsRepresentation = std::make_shared<FixedRepresentation>("Physics");
	physicsRepresentation->setShape(meshShape);

	auto  collisionRepresentation = std::make_shared<RigidCollisionRepresentation>("Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	element->addComponent(meshShapeVisualization);
	element->addComponent(collisionRepresentation);
	element->addComponent(physicsRepresentation);

	return element;
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

std::shared_ptr<OsgViewElement> createViewElement()
{
	auto result = std::make_shared<OsgViewElement>("View");
	result->enableManipulator(true);
	result->setManipulatorParameters(Vector3d(0.0, 0.35, 0.35), Vector3d::Zero());
	result->enableKeyboardDevice(true);
	result->setPose(
		SurgSim::Math::makeRigidTransform(Vector3d(1.0, 1.0, 1.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0)));

	result->getCamera()->setAmbientColor(Vector4d(0.2, 0.2, 0.2, 1.0));
	result->setPose(makeRigidTransform(Vector3d(0.0, 0.5, 0.5), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0)));

	// Move the light from left to right over along the scene
	auto interpolator = std::make_shared<SurgSim::Blocks::PoseInterpolator>("Interpolator");
	RigidTransform3d from = makeRigidTransform(Vector3d(1.0, 1.0, 0.5),
							Vector3d(0.0, 0.0, 0.0),
							Vector3d(0.0, 1.0, 0.0));
	RigidTransform3d to = makeRigidTransform(Vector3d(-1.0, 1.0, 0.5),
						  Vector3d(0.0, 0.0, 0.0),
						  Vector3d(0.0, 1.0, 0.0));
	interpolator->setTarget(result);
	interpolator->setStartingPose(from);
	interpolator->setDuration(2.5);
	interpolator->setEndingPose(to);
	interpolator->setPingPong(true);

	// result->addComponent(interpolator);


	return result;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createLightElement()
{
	auto result = std::make_shared<SurgSim::Framework::BasicSceneElement>("Light");

	auto light = std::make_shared<SurgSim::Graphics::OsgLight>("Light");
	light->setDiffuseColor(Vector4d(0.9, 0.9, 0.9, 1.0));
	light->setSpecularColor(Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);
	result->addComponent(light);

	// Move the light from left to right over along the scene
	auto interpolator = std::make_shared<SurgSim::Blocks::PoseInterpolator>("Interpolator");
	RigidTransform3d from = makeRigidTransform(Vector3d(0.1, 1.0, 0.0),
							Vector3d(0.0, 0.0, 0.0),
							Vector3d(0.0, 1.0, 0.0));
	RigidTransform3d to = makeRigidTransform(Vector3d(0.1, 1.0, 0.0),
						  Vector3d(0.0, 0.0, 0.0),
						  Vector3d(0.0, 1.0, 0.0));
	interpolator->setTarget(result);
	interpolator->setStartingPose(from);
	interpolator->setDuration(10.0);
	interpolator->setEndingPose(to);
	interpolator->setPingPong(true);

	auto component = std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("axes");
	component->setSize(0.1);
	result->addComponent(component);

	result->setPose(from);

	result->addComponent(interpolator);

	return result;
}

int main(int argc, char* argv[])
{
	const std::string deviceName = "MultiAxisDevice";
	SurgSim::Framework::Logger::getLogger("Physics/VirtualToolCoupler")->setThreshold(
		SurgSim::Framework::LOG_LEVEL_INFO);
	SurgSim::Framework::Logger::getLogger("Physics Manager")->setThreshold(SurgSim::Framework::LOG_LEVEL_INFO);

	std::shared_ptr<BehaviorManager> behaviorManager = std::make_shared<BehaviorManager>();
	std::shared_ptr<OsgManager> graphicsManager = std::make_shared<OsgManager>();
	std::shared_ptr<InputManager> inputManager = std::make_shared<InputManager>();
	std::shared_ptr<StaplingPhysicsManager> physicsManager = std::make_shared<StaplingPhysicsManager>();
	physicsManager->setRate(170.0);

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>("config.txt");
	runtime->addManager(behaviorManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(inputManager);
	runtime->addManager(physicsManager);

	DeviceFactory deviceFactory;
	std::shared_ptr<DeviceInterface> device = deviceFactory.getDevice(deviceName);
	SURGSIM_ASSERT(device != nullptr) << "Unable to get a device, is one connected?";
	inputManager->addDevice(device);

	std::shared_ptr<OsgViewElement> view = createViewElement();
	inputManager->addDevice(view->getKeyboardDevice());

	// Shader should be shared between all materials using the same shader
	auto shader = SurgSim::Graphics::loadProgram(*runtime->getApplicationData(), "Shaders/ds_mapping_material");
	SURGSIM_ASSERT(shader != nullptr) << "Shader could not be loaded.";


	std::shared_ptr<SceneElement> arm = createArmSceneElement("arm");

	std::shared_ptr<SceneElement> stapler = createStaplerSceneElement("stapler", deviceName);
	RigidTransform3d offset = makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.2, 0.0));
	stapler->setPose(offset);

	std::string woundFilename = std::string("Geometry/wound_deformable.ply");
	// Mechanical properties are based on Liang and Boppart, "Biomechanical Properties of In Vivo Human Skin From
	// Dynamic Optical Coherence Elastography", IEEE Transactions on Biomedical Engineering, Vol 57, No 4.

	// Material for the wound
	auto material = SurgSim::Blocks::createNormalMappedMaterial("woundmaterial",
					Vector4f(1.0, 1.0, 1.0, 1.0),
					Vector4f(0.4, 0.4, 0.4, 1.0), 10.0,
					"Geometry/wound.png",
					"Geometry/wound_normal.png");

	std::shared_ptr<SceneElement> wound =
		createFemSceneElement("wound",
							  woundFilename,
							  SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER,
							  material);

	std::shared_ptr<InputComponent> keyboardComponent = std::make_shared<InputComponent>("KeyboardInputComponent");
	keyboardComponent->setDeviceName("Keyboard"); // Name of device is case sensitive.
	std::shared_ptr<KeyboardTogglesComponentBehavior> keyboardBehavior =
		std::make_shared<KeyboardTogglesComponentBehavior>("KeyboardBehavior");
	keyboardBehavior->setInputComponent(keyboardComponent);


	// This should be changed to do lookup by name rather than use direct references, it will become easier to
	// setup
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_A, stapler->getComponent("Handle"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_A, stapler->getComponent("Stapler"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_A, stapler->getComponent("Footplate"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_B, stapler->getComponent("Contacts"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_C, stapler->getComponent("Collision Mesh"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_D, arm->getComponent("Forearm"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_D, arm->getComponent("Upperarm"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_E, arm->getComponent("Collision Mesh"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_F, wound->getComponent("Graphics"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_G, wound->getComponent("Wire Frame"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_H, stapler->getComponent("Tooth Graphics0"));
	keyboardBehavior->registerKey(SurgSim::Device::KeyCode::KEY_H, stapler->getComponent("Tooth Graphics1"));

	auto quitter = std::make_shared<KeyToQuitBehavior>("Quitter");
	quitter->setCallback(stopRunning);
	quitter->setInputComponent(keyboardComponent);

	auto keyboard = std::make_shared<BasicSceneElement>("Keyboard");
	keyboard->addComponent(keyboardComponent);
	keyboard->addComponent(keyboardBehavior);
	keyboard->addComponent(quitter);





	std::shared_ptr<Scene> scene = runtime->getScene();
	scene->addSceneElement(view);
	scene->addSceneElement(stapler);
	scene->addSceneElement(arm);
	scene->addSceneElement(wound);
	scene->addSceneElement(keyboard);
	scene->addSceneElement(createLightElement());

	runtime->addSceneElements("Scenery.yaml");

	auto materials = createMaterials(runtime->getScene());
	SurgSim::Blocks::applyMaterials(runtime->getScene(), "Materials.yaml", materials);

#undef STAPLING_SHADOWS
#ifdef STAPLING_SHADOWS
	setupShadowMapping(materials, scene);
#else
	view->getCamera()->setMaterial(materials["placeholder"]);
#endif

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

	run(runtime);

	return 0;
}

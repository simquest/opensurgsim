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
#include <boost/exception/to_string.hpp>

#include "Examples/ExampleStapling/StaplerBehavior.h"

#include "SurgSim/Blocks/KeyboardTogglesGraphicsBehavior.h"
#include "SurgSim/Blocks/TransferOdeStateToVerticesBehavior.h"
#include "SurgSim/Blocks/VisualizeContactsBehavior.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/MeshRepresentation.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgShader.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"
#include "SurgSim/Graphics/MeshPlyReaderDelegate.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgLight.h"


using SurgSim::Blocks::KeyboardTogglesGraphicsBehavior;
using SurgSim::Blocks::VisualizeContactsBehavior;
using SurgSim::Collision::ShapeCollisionRepresentation;
using SurgSim::DataStructures::EmptyData;
using SurgSim::Device::IdentityPoseDevice;
using SurgSim::DataStructures::PlyReader;
using SurgSim::DataStructures::TriangleMeshBase;
using SurgSim::DataStructures::TriangleMeshPlyReaderDelegate;
using SurgSim::Device::MultiAxisDevice;
using SurgSim::Framework::ApplicationData;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::Mesh;
using SurgSim::Graphics::MeshRepresentation;
using SurgSim::Graphics::SceneryRepresentation;
using SurgSim::Graphics::OsgManager;
using SurgSim::Graphics::OsgMeshRepresentation;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::OsgSceneryRepresentation;
using SurgSim::Math::MeshShape;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationMatrix;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Vector4f;
using SurgSim::Input::DeviceInterface;
using SurgSim::Input::InputComponent;
using SurgSim::Input::InputManager;
using SurgSim::Physics::DeformableCollisionRepresentation;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::PhysicsManager;
using SurgSim::Physics::RigidRepresentationParameters;
using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::VirtualToolCoupler;

namespace
{
std::unordered_map<std::string, std::shared_ptr<SurgSim::Graphics::OsgShader>> shaders;
}

static std::shared_ptr<TriangleMeshBase<EmptyData, EmptyData, EmptyData>> loadMesh(const std::string& fileName)
{
	// The PlyReader and TriangleMeshPlyReaderDelegate work together to load triangle meshes.
	SurgSim::DataStructures::PlyReader reader(fileName);
	std::shared_ptr<SurgSim::DataStructures::TriangleMeshPlyReaderDelegate> triangleMeshDelegate
		= std::make_shared<SurgSim::DataStructures::TriangleMeshPlyReaderDelegate>();

	SURGSIM_ASSERT(reader.setDelegate(triangleMeshDelegate)) << "The input file " << fileName << " is malformed.";
	reader.parseFile();

	return triangleMeshDelegate->getMesh();
}

static std::shared_ptr<SurgSim::Graphics::Mesh> loadGraphicsMesh(const std::string& fileName)
{
	// The PlyReader and TriangleMeshPlyReaderDelegate work together to load triangle meshes.
	SurgSim::DataStructures::PlyReader reader(fileName);
	auto delegate = std::make_shared<SurgSim::Graphics::MeshPlyReaderDelegate>();
	SURGSIM_ASSERT(reader.setDelegate(delegate)) << "The input file " << fileName << " is malformed.";
	reader.parseFile();

	return delegate->getMesh();
}

static std::shared_ptr<SurgSim::Framework::SceneElement> createFemSceneElement(
	const std::string& name,
	const std::string& filename,
	SurgSim::Math::IntegrationScheme integrationScheme,
	bool displayPointCloud,
	std::shared_ptr<SurgSim::Graphics::OsgMaterial> material)
{
	// Create a SceneElement that bundles the pieces associated with the finite element model
	std::shared_ptr<SurgSim::Framework::SceneElement> sceneElement
		= std::make_shared<SurgSim::Framework::BasicSceneElement>(name);

	// Load the tetrahedral mesh and initialize the finite element model
	std::shared_ptr<SurgSim::Physics::Fem3DRepresentation> physicsRepresentation
		= std::make_shared<SurgSim::Physics::Fem3DRepresentation>(name + " physics");
	physicsRepresentation->setFilename(filename);
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	// Note: Directly calling loadFile is a workaround.  The TransferOdeStateToVerticesBehavior requires a
	// pointer to the Physics Representation's state, which is not created until the file is loaded and the internal
	// structure is initialized.  Therefore we create the state now by calling loadFile.
	physicsRepresentation->loadFile();
	sceneElement->addComponent(physicsRepresentation);

	// Create a triangle mesh for visualizing the surface of the finite element model
	std::shared_ptr<SurgSim::Graphics::OsgMeshRepresentation> graphicsTriangleMeshRepresentation
		= std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>(name + " triangle mesh");
	*graphicsTriangleMeshRepresentation->getMesh() = SurgSim::Graphics::Mesh(*loadGraphicsMesh(filename));
	sceneElement->addComponent(graphicsTriangleMeshRepresentation);

	// Create material to transport the Textures
	graphicsTriangleMeshRepresentation->setMaterial(material);

	// Load the surface triangle mesh of the finite element model
	auto meshSurface = loadMesh(filename);

	// Create the collision mesh for the surface of the finite element model
	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision");
	collisionRepresentation->setMesh(std::make_shared<SurgSim::DataStructures::TriangleMesh>(*meshSurface));
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);
	sceneElement->addComponent(collisionRepresentation);

	// Create a behavior which transfers the position of the vertices in the FEM to locations in the triangle mesh
	sceneElement->addComponent(
		std::make_shared<SurgSim::Blocks::TransferOdeStateToVerticesBehavior<SurgSim::Graphics::VertexData>>(
			name + " physics to triangle mesh",
			physicsRepresentation->getFinalState(),
			graphicsTriangleMeshRepresentation->getMesh()));

	if (displayPointCloud)
	{
		// Create a point-cloud for visualizing the nodes of the finite element model
		std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation<EmptyData>> graphicsPointCloudRepresentation
				= std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<EmptyData>>(name + " point cloud");
		graphicsPointCloudRepresentation->setColor(SurgSim::Math::Vector4d(1.0, 1.0, 1.0, 1.0));
		graphicsPointCloudRepresentation->setPointSize(3.0f);
		graphicsPointCloudRepresentation->setVisible(true);
		sceneElement->addComponent(graphicsPointCloudRepresentation);

		// Create a behavior which transfers the position of the vertices in the FEM to locations in the point cloud
		sceneElement->addComponent(
			std::make_shared<SurgSim::Blocks::TransferOdeStateToVerticesBehavior<EmptyData>>(
				name + " physics to point cloud",
				physicsRepresentation->getFinalState(),
				graphicsPointCloudRepresentation->getVertices()));
	}

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
	auto data = std::make_shared<SurgSim::Framework::ApplicationData>("config.txt");

	// Stapler collision mesh
	std::shared_ptr<MeshShape> meshShapeForCollision = std::make_shared<MeshShape>();
	meshShapeForCollision->setFileName("Geometry/stapler_collision.ply");
	meshShapeForCollision->initialize(*data);

	std::shared_ptr<MeshRepresentation> meshShapeVisualization =
		std::make_shared<OsgMeshRepresentation>("StaplerOsgMesh");
	*meshShapeVisualization->getMesh() = SurgSim::Graphics::Mesh(*(meshShapeForCollision->getMesh()));
	meshShapeVisualization->setDrawAsWireFrame(true);
	meshShapeVisualization->setVisible(false);

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
	meshShapeForVirtualStaple1->setFileName("Geometry/virtual_staple_1.ply");
	meshShapeForVirtualStaple1->initialize(*data);

	auto meshShapeForVirtualStaple2 = std::make_shared<MeshShape>();
	meshShapeForVirtualStaple2->setFileName("Geometry/virtual_staple_2.ply");
	meshShapeForVirtualStaple2->initialize(*data);

	std::vector<std::shared_ptr<MeshShape>> virtualTeethShapes;
	virtualTeethShapes.push_back(meshShapeForVirtualStaple1);
	virtualTeethShapes.push_back(meshShapeForVirtualStaple2);

	int i = 0;
	std::array<std::shared_ptr<SurgSim::Collision::Representation>, 2> virtualTeeth;
	for (auto it = virtualTeethShapes.begin(); it != virtualTeethShapes.end(); ++it, ++i)
	{
		std::shared_ptr<ShapeCollisionRepresentation> virtualToothCollision =
			std::make_shared<SurgSim::Collision::ShapeCollisionRepresentation>(
				"VirtualToothCollision" + boost::to_string(i));
		virtualToothCollision->setShape(*it);
		virtualToothCollision->setLocalPose(RigidTransform3d::Identity());

		virtualTeeth[i] = virtualToothCollision;
		sceneElement->addComponent(virtualToothCollision);

		std::shared_ptr<OsgMeshRepresentation> virtualToothMesh
			= std::make_shared<OsgMeshRepresentation>("virtualToothMesh" + boost::to_string(i));
		*virtualToothMesh->getMesh() = SurgSim::Graphics::Mesh(*(*it)->getMesh());
		virtualToothMesh->setDrawAsWireFrame(true);

		sceneElement->addComponent(virtualToothMesh);
	}

	staplerBehavior->setVirtualTeeth(virtualTeeth);

	return sceneElement;
}

std::shared_ptr<SceneElement> createArmSceneElement(
	const std::string& armName,
	std::shared_ptr<SurgSim::Graphics::OsgMaterial> material)
{
	auto data = std::make_shared<SurgSim::Framework::ApplicationData>("config.txt");

	// Graphic representation for arm
	std::shared_ptr<SceneryRepresentation> forearmSceneryRepresentation =
		createSceneryObject("forearm", "Geometry/forearm.osgb");
	forearmSceneryRepresentation->setMaterial(material);
	std::shared_ptr<SceneryRepresentation> upperarmSceneryRepresentation =
		createSceneryObject("upperarm", "Geometry/upperarm.osgb");
	upperarmSceneryRepresentation->setMaterial(material);

	// Arm collision mesh
	std::shared_ptr<MeshShape> meshShape = std::make_shared<MeshShape>();
	meshShape->setFileName("Geometry/arm_collision.ply");
	meshShape->initialize(*data);

	std::shared_ptr<MeshRepresentation> meshShapeVisualization = std::make_shared<OsgMeshRepresentation>("ArmOsgMesh");
	*meshShapeVisualization->getMesh() = SurgSim::Graphics::Mesh(*(meshShape->getMesh()));
	meshShapeVisualization->setDrawAsWireFrame(true);
	meshShapeVisualization->setVisible(false);

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

std::shared_ptr<OsgViewElement> createViewElement(const ApplicationData& data)
{
	auto result = std::make_shared<OsgViewElement>("StaplingDemoView");
	result->enableManipulator(true);
	result->setManipulatorParameters(Vector3d(0.0, 0.5, 0.5), Vector3d::Zero());
	result->enableKeyboardDevice(true);

	auto light = std::make_shared<SurgSim::Graphics::OsgLight>("Light");
	light->setDiffuseColor(Vector4d(1.0, 1.0, 1.0, 1.0));
	light->setSpecularColor(Vector4d(1.0, 1.0, 1.0, 1.0));
	result->addComponent(light);

	result->getCamera()->setAmbientColor(Vector4d(0.2, 0.2, 0.2, 1.0));

	return result;
}

std::shared_ptr<SurgSim::Graphics::OsgMaterial> createShinyMaterial(
	const SurgSim::Framework::ApplicationData& data,
	std::string defaultTextureName = "Textures/checkered.png")
{
	// Default Material with shader
	// using scopes to keep from having to introduce new variables with different types
	auto material = std::make_shared<SurgSim::Graphics::OsgMaterial>();
	material->setShader(shaders["shinytextured"]);

	SURGSIM_ASSERT(material != nullptr) << "Material could not be loaded !";
	{
		auto uniform = std::make_shared<SurgSim::Graphics::OsgUniform<Vector4f>>("diffuseColor");
		material->addUniform(uniform);
		material->setValue("diffuseColor", SurgSim::Math::Vector4f(1.0, 1.0, 1.0, 1.0));
	}

	{
		auto uniform = std::make_shared<SurgSim::Graphics::OsgUniform<Vector4f>>("specularColor");
		material->addUniform(uniform);
		material->setValue("specularColor", SurgSim::Math::Vector4f(0.01, 0.01, 0.01, 1.0));
	}

	{
		auto uniform = std::make_shared<SurgSim::Graphics::OsgUniform<float>>("shininess");
		material->addUniform(uniform);
		material->setValue("shininess", 32.0f);
	}

	std::string blackTexture;
	SURGSIM_ASSERT(data.tryFindFile("Textures/black.png", &blackTexture));

	std::string defaultTexture;
	SURGSIM_ASSERT(data.tryFindFile(defaultTextureName, &defaultTexture));

	{
		// As a default color for the texture map use white
		auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
		texture->loadImage(defaultTexture);
		auto uniform =
			std::make_shared<SurgSim::Graphics::OsgTextureUniform<SurgSim::Graphics::OsgTexture2d>>("diffuseMap");
		uniform->set(texture);
		material->addUniform(uniform);
	}

	{
		// The neutral color for the shadow map is black
		auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
		texture->loadImage(blackTexture);
		auto uniform =
			std::make_shared<SurgSim::Graphics::OsgTextureUniform<SurgSim::Graphics::OsgTexture2d>>("shadowMap");
		uniform->set(texture);
		uniform->setMinimumTextureUnit(8);
		material->addUniform(uniform);
	}

	return material;
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

	std::shared_ptr<OsgViewElement> view = createViewElement(*runtime->getApplicationData());
	inputManager->addDevice(view->getKeyboardDevice());

	// Shader should be shared between all materials using the same shader
	auto shader = SurgSim::Graphics::loadShader(*runtime->getApplicationData(), "Shaders/ds_mapping_material");

	RigidTransform3d armPose = makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, -0.2, 0.0));
	auto material = createShinyMaterial(*runtime->getApplicationData());
	material->setShader(shader);
	std::shared_ptr<SceneElement> arm = createArmSceneElement("arm", material);
	arm->setPose(armPose);

	std::shared_ptr<SceneElement> stapler = createStaplerSceneElement("stapler", deviceName);
	stapler->setPose(RigidTransform3d::Identity());

	// Load the FEM
	std::string woundFilename = runtime->getApplicationData()->findFile("Geometry/wound_deformable.ply");
	// Mechanical properties are based on Liang and Boppart, "Biomechanical Properties of In Vivo Human Skin From
	// Dynamic Optical Coherence Elastography", IEEE Transactions on Biomedical Engineering, Vol 57, No 4.


	// Material for the wound
	material = createShinyMaterial(*runtime->getApplicationData(), "Geometry/wound.png");
	material->setShader(shader);

	bool doPointCloud = false;
	std::shared_ptr<SceneElement> wound =
		createFemSceneElement("wound",
							  woundFilename,
							  SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER,
							  false,
							  material);
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
		SurgSim::Device::KeyCode::KEY_E, wound->getComponent("wound triangle mesh"));

	if (doPointCloud)
	{
		keyboardBehavior->registerKey(
			SurgSim::Device::KeyCode::KEY_F, wound->getComponent("wound point cloud"));
	}

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

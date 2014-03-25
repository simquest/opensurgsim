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

#include "SurgSim/Blocks/TransferDeformableStateToVerticesBehavior.h"
#include "SurgSim/Blocks/TransferPoseBehavior.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#include "Examples/ExampleStapling/StaplerBehavior.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

using SurgSim::Blocks::TransferPoseBehavior;
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
using SurgSim::Graphics::SceneryRepresentation;
using SurgSim::Graphics::ViewElement;
using SurgSim::Graphics::OsgManager;
using SurgSim::Graphics::OsgMeshRepresentation;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::OsgSceneryRepresentation;
using SurgSim::Graphics::ViewElement;
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

static std::shared_ptr<SurgSim::Physics::Fem3DRepresentation> loadFem(
	const std::string& fileName,
	SurgSim::Math::IntegrationScheme integrationScheme,
	double massDensity,
	double poissonRatio,
	double youngModulus)
{
	// The PlyReader and Fem3DRepresentationPlyReaderDelegate work together to load 3d fems.
	SurgSim::DataStructures::PlyReader reader(fileName);
	std::shared_ptr<SurgSim::Physics::Fem3DRepresentationPlyReaderDelegate> fem3dDelegate
		= std::make_shared<SurgSim::Physics::Fem3DRepresentationPlyReaderDelegate>();

	SURGSIM_ASSERT(reader.setDelegate(fem3dDelegate)) << "The input file " << fileName << " is malformed.";
	reader.parseFile();

	std::shared_ptr<SurgSim::Physics::Fem3DRepresentation> fem = fem3dDelegate->getFem();

	// The FEM requires the implicit Euler integration scheme to avoid "blowing up"
	fem->setIntegrationScheme(integrationScheme);

	// Physical parameters must be set for the finite elements in order to be valid for the simulation.
	for (size_t i = 0; i < fem->getNumFemElements(); i++)
	{
		fem->getFemElement(i)->setMassDensity(massDensity);
		fem->getFemElement(i)->setPoissonRatio(poissonRatio);
		fem->getFemElement(i)->setYoungModulus(youngModulus);
	}

	return fem;
}

static std::shared_ptr<SurgSim::Framework::SceneElement> createFemSceneElement(
	const std::string& name,
	const std::string& filename,
	SurgSim::Math::IntegrationScheme integrationScheme,
	double massDensity,
	double poissonRatio,
	double youngModulus,
	bool displayPointCloud,
	const SurgSim::Math::RigidTransform3d& pose)
{
	// Create a SceneElement that bundles the pieces associated with the finite element model
	std::shared_ptr<SurgSim::Framework::SceneElement> sceneElement
		= std::make_shared<SurgSim::Framework::BasicSceneElement>(name);

	// Load the tetrahedral mesh and initialize the finite element model
	std::shared_ptr<SurgSim::Physics::Fem3DRepresentation> physicsRepresentation
		= loadFem(filename, integrationScheme, massDensity, poissonRatio, youngModulus);
	physicsRepresentation->setInitialPose(pose);
	sceneElement->addComponent(physicsRepresentation);

	// Create a triangle mesh for visualizing the surface of the finite element model
	std::shared_ptr<SurgSim::Graphics::OsgMeshRepresentation> graphicsTriangleMeshRepresentation
		= std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>(name + " triangle mesh");
	*graphicsTriangleMeshRepresentation->getMesh() = SurgSim::Graphics::Mesh(*loadMesh(filename));
	graphicsTriangleMeshRepresentation->setInitialPose(pose);
	sceneElement->addComponent(graphicsTriangleMeshRepresentation);

	// Create a behavior which transfers the position of the vertices in the FEM to locations in the triangle mesh
	sceneElement->addComponent(
		std::make_shared<SurgSim::Blocks::TransferDeformableStateToVerticesBehavior<SurgSim::Graphics::VertexData>>(
			name + " physics to triangle mesh",
			physicsRepresentation->getFinalState(),
			graphicsTriangleMeshRepresentation->getMesh()));

	if (displayPointCloud)
	{
		// Create a point-cloud for visualizing the nodes of the finite element model
		std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation<EmptyData>> graphicsPointCloudRepresentation
			= std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<EmptyData>>(name + " point cloud");
		graphicsPointCloudRepresentation->setInitialPose(SurgSim::Math::RigidTransform3d::Identity());
		graphicsPointCloudRepresentation->setColor(SurgSim::Math::Vector4d(1.0, 1.0, 1.0, 1.0));
		graphicsPointCloudRepresentation->setPointSize(3.0f);
		graphicsPointCloudRepresentation->setVisible(true);
		graphicsPointCloudRepresentation->setInitialPose(pose);
		sceneElement->addComponent(graphicsPointCloudRepresentation);

		// Create a behavior which transfers the position of the vertices in the FEM to locations in the point cloud
		sceneElement->addComponent(
			std::make_shared<SurgSim::Blocks::TransferDeformableStateToVerticesBehavior<EmptyData>>(
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

std::shared_ptr<ViewElement> createView()
{
	std::shared_ptr<OsgViewElement> view = std::make_shared<OsgViewElement>("StaplingDemoView");

	view->enableManipulator(true);
	view->setManipulatorParameters(Vector3d(0.0, 0.5, 0.5), Vector3d::Zero());

	return view;
}

std::shared_ptr<SceneElement> createStaplerSceneElement(const std::string& staplerName,
														const std::string& deviceName,
														const SurgSim::Math::RigidTransform3d& pose)
{
	std::vector<std::shared_ptr<SurgSim::Framework::Representation>> recievesPhysicsPose;

	std::vector<std::string> paths;
	paths.push_back("Data/Geometry");
	ApplicationData data(paths);

	std::shared_ptr<TriangleMeshPlyReaderDelegate> delegate = std::make_shared<TriangleMeshPlyReaderDelegate>();
	PlyReader reader(data.findFile("stapler_collision.ply"));
	reader.setDelegate(delegate);
	reader.parseFile();

	std::shared_ptr<OsgMeshRepresentation> osgMeshRepresentation =
		std::make_shared<OsgMeshRepresentation>("StaplerOsgMesh");
	*osgMeshRepresentation->getMesh() = SurgSim::Graphics::Mesh(*delegate->getMesh());
	osgMeshRepresentation->setInitialPose(pose);
	osgMeshRepresentation->setDrawAsWireFrame(true);
	recievesPhysicsPose.push_back(osgMeshRepresentation);

	// Stapler collision mesh
	std::shared_ptr<MeshShape> meshShape = std::make_shared<MeshShape>(*delegate->getMesh()); // Unit: meter
	RigidRepresentationParameters params;
	params.setDensity(8050); // Stainless steel (in Kg.m-3)
	params.setShapeUsedForMassInertia(meshShape);

	std::shared_ptr<RigidRepresentation> physicsRepresentation = std::make_shared<RigidRepresentation>("Physics");
	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setIsGravityEnabled(false);
	physicsRepresentation->setInitialPose(pose);

	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("Collision");
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
	std::shared_ptr<StaplerBehavior> staplerBehavior = std::make_shared<StaplerBehavior>("Behavior");
	staplerBehavior->setInputComponent(inputComponent);
	staplerBehavior->setCollisionRepresentation(collisionRepresentation);

	std::shared_ptr<SceneElement> sceneElement = std::make_shared<BasicSceneElement>(staplerName + "SceneElement");
	sceneElement->addComponent(physicsRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(osgMeshRepresentation);
	sceneElement->addComponent(inputComponent);
	sceneElement->addComponent(inputVTC);
	sceneElement->addComponent(staplerBehavior);

	// Load the graphical parts of a stapler.
	std::list<std::shared_ptr<SceneryRepresentation>> sceneryRepresentations;
	sceneryRepresentations.push_back(createSceneryObject("Handle",    "Geometry/stapler_handle.obj"));
	sceneryRepresentations.push_back(createSceneryObject("Indicator", "Geometry/stapler_indicator.obj"));
	sceneryRepresentations.push_back(createSceneryObject("Markings",  "Geometry/stapler_markings.obj"));
	sceneryRepresentations.push_back(createSceneryObject("Trigger",   "Geometry/stapler_trigger.obj"));
	for (auto it = std::begin(sceneryRepresentations); it != std::end(sceneryRepresentations); ++it)
	{
		(*it)->setInitialPose(pose);

		recievesPhysicsPose.push_back(*it);
		sceneElement->addComponent(*it);
	}

	std::vector<std::shared_ptr<MeshShape>> virtualTeethShapes;
	virtualTeethShapes.push_back(std::make_shared<MeshShape>(*loadMesh("Data/Geometry/virtual_staple_1.ply")));
	virtualTeethShapes.push_back(std::make_shared<MeshShape>(*loadMesh("Data/Geometry/virtual_staple_2.ply")));

	int i = 0;
	std::array<std::shared_ptr<ShapeCollisionRepresentation>, 2> virtualTeeth;
	for (auto it = virtualTeethShapes.begin(); it != virtualTeethShapes.end(); ++it)
	{
		std::shared_ptr<ShapeCollisionRepresentation> virtualToothCollision
			= std::make_shared<SurgSim::Collision::ShapeCollisionRepresentation>(
				"VirtualToothCollision" + boost::to_string(i), *it, RigidTransform3d::Identity());

		virtualTeeth[i] = virtualToothCollision;
		sceneElement->addComponent(virtualToothCollision);
		recievesPhysicsPose.push_back(virtualToothCollision);

		std::shared_ptr<OsgMeshRepresentation> virtualToothMesh
			= std::make_shared<OsgMeshRepresentation>("virtualToothMesh" + boost::to_string(i));
		*virtualToothMesh->getMesh() = SurgSim::Graphics::Mesh(*(*it)->getMesh());
		virtualToothMesh->setInitialPose(pose);
		virtualToothMesh->setDrawAsWireFrame(true);

		sceneElement->addComponent(virtualToothMesh);
		recievesPhysicsPose.push_back(virtualToothMesh);

		i++;
	}

	staplerBehavior->setVirtualStaple(virtualTeeth[0], virtualTeeth[1]);

	for (auto it = recievesPhysicsPose.begin(); it != recievesPhysicsPose.end(); ++it)
	{
		std::shared_ptr<TransferPoseBehavior> transferPose
			= std::make_shared<TransferPoseBehavior>("Physics to " + (*it)->getName());
		transferPose->setPoseSender(physicsRepresentation);
		transferPose->setPoseReceiver(*it);
		sceneElement->addComponent(transferPose);
	}

	return sceneElement;
}
std::shared_ptr<SceneElement> createArmSceneElement(const std::string& armName, const RigidTransform3d& pose)
{
	std::vector<std::string> paths;
	paths.push_back("Data/Geometry");
	ApplicationData data(paths);

	// File "arm_collision.ply" contains collision mesh for upper arm and forearm.
	std::shared_ptr<TriangleMeshPlyReaderDelegate> delegate = std::make_shared<TriangleMeshPlyReaderDelegate>();
	PlyReader reader(data.findFile("arm_collision.ply"));
	reader.setDelegate(delegate);
	reader.parseFile();

	std::shared_ptr<OsgMeshRepresentation> osgMeshRepresentation =
		std::make_shared<OsgMeshRepresentation>("ArmOsgMesh");
	*osgMeshRepresentation->getMesh() = SurgSim::Graphics::Mesh(*delegate->getMesh());
	osgMeshRepresentation->setInitialPose(pose);
	osgMeshRepresentation->setDrawAsWireFrame(true);

	// Graphic representation for arm
	std::shared_ptr<SceneryRepresentation> forearmSceneryRepresentation =
		createSceneryObject("forearm", "Geometry/forearm.osgb");
	forearmSceneryRepresentation->setInitialPose(pose);
	std::shared_ptr<SceneryRepresentation> upperarmSceneryRepresentation =
		createSceneryObject("upperarm", "Geometry/upperarm.osgb");
	upperarmSceneryRepresentation->setInitialPose(pose);

	// MeshShape collision representation of the arm.
	std::shared_ptr<MeshShape> meshShape = std::make_shared<MeshShape>(*delegate->getMesh());
	RigidRepresentationParameters params;
	params.setShapeUsedForMassInertia(meshShape);

	std::shared_ptr<FixedRepresentation> physicsRepresentation = std::make_shared<FixedRepresentation>("Physics");
	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setInitialPose(pose);

	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("Collision");
	collisionRepresentation->setRigidRepresentation(physicsRepresentation);

	std::shared_ptr<SceneElement> armSceneElement = std::make_shared<BasicSceneElement>(armName + "SceneElement");
	armSceneElement->addComponent(forearmSceneryRepresentation);
	armSceneElement->addComponent(osgMeshRepresentation);
	armSceneElement->addComponent(upperarmSceneryRepresentation);
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

	std::shared_ptr<DeviceInterface> device;
	device = std::make_shared<MultiAxisDevice>(deviceName);
	if (!device->initialize())
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
			<< "Could not initialize device " << device->getName() << " for the tool.";

		device = std::make_shared<IdentityPoseDevice>(deviceName);
	}
	inputManager->addDevice(device);

	std::shared_ptr<Scene> scene = runtime->getScene();
	RigidTransform3d armPose = makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, -0.2, 0.0));
	scene->addSceneElement(createView());
	scene->addSceneElement(createArmSceneElement("arm", armPose));
	scene->addSceneElement(createStaplerSceneElement(
		"stapler", deviceName, makeRigidTransform(Quaterniond::Identity(), Vector3d::Zero())));

	// Load the FEM
	std::string woundFilename = runtime->getApplicationData()->findFile("Geometry/wound_deformable.ply");
	// Mechanical properties are based on Liang and Boppart, "Biomechanical Properties of In Vivo Human Skin From
	// Dynamic Optical Coherence Elastography", IEEE Transactions on Biomedical Engineering, Vol 57, No 4.
	scene->addSceneElement(
		createFemSceneElement("wound",
							  woundFilename,
							  SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER, // Physics loop update technique
							  1000.0,										   // Mass Density
							  0.45,											   // Poisson Ratio
							  75e3,											   // Young Modulus
							  true,											   // Display point cloud
							  armPose));									   // Pose of wound on arm
	runtime->execute();

	return 0;
}

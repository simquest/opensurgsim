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

#include "SurgSim/Blocks/TransferDeformableStateToVerticesBehavior.h"
#include "SurgSim/Blocks/TransferPoseBehavior.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/PlyReader.h"
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
#include "SurgSim/Graphics/OsgCapsuleRepresentation.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Math/CapsuleShape.h"
#include "SurgSim/Math/SphereShape.h"
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
using SurgSim::DataStructures::EmptyData;
using SurgSim::Device::IdentityPoseDevice;
using SurgSim::Device::MultiAxisDevice;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::CapsuleRepresentation;
using SurgSim::Graphics::SceneryRepresentation;
using SurgSim::Graphics::SphereRepresentation;
using SurgSim::Graphics::ViewElement;
using SurgSim::Graphics::OsgCapsuleRepresentation;
using SurgSim::Graphics::OsgSphereRepresentation;
using SurgSim::Graphics::OsgManager;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::OsgSceneryRepresentation;
using SurgSim::Graphics::ViewElement;
using SurgSim::Math::CapsuleShape;
using SurgSim::Math::SphereShape;
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

static std::string findFile(std::string fileName)
{
	// Helper function for locating files in the correct folder.
	std::vector<std::string> paths;
	paths.push_back("Data/Geometry");
	SurgSim::Framework::ApplicationData data(paths);

	return data.findFile(fileName);
}

static std::shared_ptr<SurgSim::Graphics::Mesh> graphicsMeshFromTriMesh(
	std::shared_ptr<SurgSim::DataStructures::TriangleMeshBase<EmptyData, EmptyData, EmptyData>> mesh)
{
	std::shared_ptr<SurgSim::Graphics::Mesh> result = std::make_shared<SurgSim::Graphics::Mesh>();

	// Graphics::Mesh requires specific parameters for initialization
	std::vector<SurgSim::Math::Vector4d> emptyColors;
	std::vector<SurgSim::Math::Vector2d> emptyTextures;
	std::vector<SurgSim::Math::Vector3d> vertices;
	std::vector<unsigned int> triangles;

	typedef std::vector<SurgSim::DataStructures::Vertex<EmptyData>>::iterator VertexIterator;
	for (VertexIterator it = std::begin(mesh->getVertices()); it != std::end(mesh->getVertices()); ++it)
	{
		vertices.emplace_back(it->position);
	}

	typedef std::vector<SurgSim::DataStructures::MeshElement<3u, EmptyData>>::iterator TriangleIterator;
	for (TriangleIterator it = std::begin(mesh->getTriangles()); it != std::end(mesh->getTriangles()); ++it)
	{
		triangles.push_back(it->verticesId[0]);
		triangles.push_back(it->verticesId[1]);
		triangles.push_back(it->verticesId[2]);
	}

	result->initialize(vertices, emptyColors, emptyTextures, triangles);

	return result;
}

static std::shared_ptr<SurgSim::Graphics::Mesh> loadMesh(const std::string& fileName)
{
	std::string fileFullName = findFile(fileName);

	// The PlyReader and TriangleMeshPlyReaderDelegate work together to load triangle meshes.
	SurgSim::DataStructures::PlyReader reader(fileFullName);
	std::shared_ptr<SurgSim::DataStructures::TriangleMeshPlyReaderDelegate> triangleMeshDelegate
		= std::make_shared<SurgSim::DataStructures::TriangleMeshPlyReaderDelegate>();

	SURGSIM_ASSERT(reader.setDelegate(triangleMeshDelegate)) << "The input file " << fileFullName << " is malformed.";
	reader.parseFile();

	// graphicsMeshFromTriMesh transforms mesh types
	return graphicsMeshFromTriMesh(triangleMeshDelegate->getMesh());
}

static std::shared_ptr<SurgSim::Physics::Fem3DRepresentation> loadFem(
	const std::string& fileName,
	SurgSim::Math::IntegrationScheme integrationScheme,
	double massDensity,
	double poissonRatio,
	double youngModulus)
{
	std::string fileFullName = findFile(fileName);

	// The PlyReader and Fem3DRepresentationPlyReaderDelegate work together to load 3d fems.
	SurgSim::DataStructures::PlyReader reader(fileFullName);
	std::shared_ptr<SurgSim::Physics::Fem3DRepresentationPlyReaderDelegate> fem3dDelegate
		= std::make_shared<SurgSim::Physics::Fem3DRepresentationPlyReaderDelegate>();

	SURGSIM_ASSERT(reader.setDelegate(fem3dDelegate)) << "The input file " << fileFullName << " is malformed.";
	reader.parseFile();

	std::shared_ptr<SurgSim::Physics::Fem3DRepresentation> fem = fem3dDelegate->getFem();

	// The FEM requires the implicit euler integration scheme to avoid "blowing up"
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

static std::shared_ptr<SurgSim::Framework::SceneElement> createFemWound(const std::string& name, bool displayPointCloud)
{
	// Create a SceneElement that bundles the pieces associated with the finite element model
	std::shared_ptr<SurgSim::Framework::SceneElement> woundSceneElement
		= std::make_shared<SurgSim::Framework::BasicSceneElement>(name);

	// Load the tetrahedral mesh and initialize the finite element model
	std::shared_ptr<SurgSim::Physics::Fem3DRepresentation> physicsRepresentation
		= loadFem("DeformableArmWound.ply", SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER, 1000.0, 0.45, 500000.0);
	woundSceneElement->addComponent(physicsRepresentation);

	// Create a triangle mesh for visualizing the surface of the finite element model
	std::shared_ptr<SurgSim::Graphics::OsgMeshRepresentation> graphicsTriangleMeshRepresentation
		= std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>(name + " triangle mesh");
	*graphicsTriangleMeshRepresentation->getMesh() = *loadMesh("DeformableArmWoundSurfaceFromInitialMesh.ply");
	woundSceneElement->addComponent(graphicsTriangleMeshRepresentation);

	// Create a behavior which transfers the position of the vertices in the FEM to locations in the triangle mesh
	woundSceneElement->addComponent(
		std::make_shared<SurgSim::Blocks::TransferDeformableStateToVerticesBehavior<SurgSim::Graphics::VertexData>>(
			name + " physics to triangle mesh",
			physicsRepresentation->getFinalState(),
			graphicsTriangleMeshRepresentation->getMesh()));

	if (displayPointCloud)
	{
		// Create a point-cloud for visualizing the nodes of the finite element model
		std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation<void>> graphicsPointCloudRepresentation
			= std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>(name + " point cloud");
		graphicsPointCloudRepresentation->setInitialPose(SurgSim::Math::RigidTransform3d::Identity());
		graphicsPointCloudRepresentation->setColor(SurgSim::Math::Vector4d(1.0, 1.0, 1.0, 1.0));
		graphicsPointCloudRepresentation->setPointSize(3.0f);
		graphicsPointCloudRepresentation->setVisible(true);
		woundSceneElement->addComponent(graphicsPointCloudRepresentation);

		// Create a behavior which transfers the position of the vertices in the FEM to locations in the point cloud
		woundSceneElement->addComponent(
			std::make_shared<SurgSim::Blocks::TransferDeformableStateToVerticesBehavior<void>>(
				name + " physics to point cloud",
				physicsRepresentation->getFinalState(),
				graphicsPointCloudRepresentation->getVertices()));
	}

	return woundSceneElement;
}

/// Load scenery object from file
/// \param name Name of this scenery representation.
/// \param fileName Name of the file from which the scenery representation is loaded.
/// \return A SceneElement containing the scenery representation.
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
	// Since there is no collision mesh loader yet, use a sphere shape as the collision representation of the stapler at
	// the tip of the stapler.
	std::shared_ptr<SphereShape> sphereShape = std::make_shared<SphereShape>(0.02); // Unit: meter
	RigidRepresentationParameters params;
	params.setDensity(8050); // Stainless steel (in Kg.m-3)
	params.setShapeUsedForMassInertia(sphereShape);

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
	inputVTC->setAngularDamping(params.getMass() * 10e-2);
	inputVTC->setAngularStiffness(params.getMass() * 50.0);
	inputVTC->setLinearDamping(params.getMass() * 10.0);
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
	// Load graphic representation for armSceneElement
	std::shared_ptr<SceneryRepresentation> sceneryRepresentation =
		createSceneryObject(armName, "Geometry/forearm.osgb");
	sceneryRepresentation->setInitialPose(pose);

	// Since there is no collision mesh loader yet, use a capsule shape as the collision representation of the arm.
	std::shared_ptr<CapsuleShape> capsuleShape = std::make_shared<CapsuleShape>(0.335, 0.03); // Unit: meter
	RigidRepresentationParameters params;
	params.setDensity(1062); // Average human body density  (in Kg.m-3)
	params.setShapeUsedForMassInertia(capsuleShape);

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

	std::shared_ptr<CapsuleRepresentation> graphicalCollisionRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("CapsuleGraphicalRepresentationOfCollision");
	graphicalCollisionRepresentation->setHeight(capsuleShape->getLength()); // Unit: meter
	graphicalCollisionRepresentation->setRadius(capsuleShape->getRadius()); // Unit: meter
	graphicalCollisionRepresentation->setInitialPose(alignedPose);

	std::shared_ptr<SceneElement> armSceneElement = std::make_shared<BasicSceneElement>("ArmSceneElement");
	armSceneElement->addComponent(sceneryRepresentation);
	armSceneElement->addComponent(collisionRepresentation);
	armSceneElement->addComponent(graphicalCollisionRepresentation);
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
	scene->addSceneElement(createView());
	scene->addSceneElement(createArm("arm", makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.0))));
	scene->addSceneElement(createStapler("stapler", deviceName));
	scene->addSceneElement(createFemWound("wound", true));

	runtime->execute();

	return 0;
}

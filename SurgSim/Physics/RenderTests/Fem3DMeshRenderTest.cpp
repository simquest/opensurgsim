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

#include <gtest/gtest.h>

#include "SurgSim/Blocks/TransferPhysicsToGraphicsMeshBehavior.h"
#include "SurgSim/Blocks/TransferPhysicsToPointCloudBehavior.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem3DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Physics
{

static std::shared_ptr<SurgSim::Framework::SceneElement> createFemSceneElement(
	const std::string& name,
	const std::string& filename,
	SurgSim::Math::IntegrationScheme integrationScheme)
{
	// Create a SceneElement that bundles the pieces associated with the finite element model
	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>(name);

	// Add the Fem3d component
	// Note that we only specify the filename that contains the full geometrical and physical description.
	auto fem = std::make_shared<SurgSim::Physics::Fem3DRepresentation>("fem3d");
	fem->loadFem(filename);
	fem->setIntegrationScheme(integrationScheme);
	sceneElement->addComponent(fem);

	auto collision = std::make_shared<SurgSim::Physics::DeformableCollisionRepresentation>("Collision");
	auto shape = std::make_shared<SurgSim::Math::MeshShape>();
	shape->load(filename);
	collision->setShape(shape);
	fem->setCollisionRepresentation(collision);
	sceneElement->addComponent(collision);

	// Add the graphics mesh used to display the Fem3d
	auto graphics = std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("fem graphics");
	graphics->loadMesh(filename);
	graphics->setDrawAsWireFrame(true);
	sceneElement->addComponent(graphics);

	// Create a behavior which transfers the position of the vertices in the FEM to locations in the triangle mesh
	auto femToMesh =
		std::make_shared<SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior>("physics to triangle mesh");
	femToMesh->setSource(fem);
	femToMesh->setTarget(graphics);
	sceneElement->addComponent(femToMesh);

	// The point-cloud for visualizing the nodes of the finite element model
	auto pointCloud
		= std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("point cloud");
	pointCloud->setColor(SurgSim::Math::Vector4d(0.2, 0.2, 1.0, 1.0));
	pointCloud->setPointSize(3.0f);
	sceneElement->addComponent(pointCloud);

	// The behavior which transfers the position of the vertices in the FEM to locations in the point cloud
	auto femToCloud = std::make_shared<SurgSim::Blocks::TransferPhysicsToPointCloudBehavior>("fem to point cloud");
	femToCloud->setSource(fem);
	femToCloud->setTarget(pointCloud);
	sceneElement->addComponent(femToCloud);

	return sceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createMeshSphere()
{
	SurgSim::Math::RigidTransform3d pose =
		SurgSim::Math::makeRigidTranslation(SurgSim::Math::Vector3d(0.0, 0.025, 0.0));

	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("RigidMesh");
	element->setPose(pose);

	auto shape = std::make_shared<SurgSim::Math::MeshShape>();
	shape->load("Geometry/sphere0_025.ply");

	auto rigid = std::make_shared<SurgSim::Physics::RigidRepresentation>("Physics");
	rigid->setIsGravityEnabled(true);
	// http://www.engineeringtoolbox.com/wood-density-d_40.html
	rigid->setDensity(5800.0); // Cedar of Lebanon wood density 5800.0 Kg/m-3
	rigid->setShape(shape);
	element->addComponent(rigid);

	auto collision = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Collision");
	//collision->setCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);
	rigid->setCollisionRepresentation(collision);
	collision->setShape(shape);
	element->addComponent(collision);

	std::shared_ptr<SurgSim::Graphics::OsgMeshRepresentation> osgRepresentation =
		std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("Graphics");
	osgRepresentation->setShape(shape);
	element->addComponent(osgRepresentation);

	return element;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createShapeSphere()
{
	SurgSim::Math::RigidTransform3d pose = SurgSim::Math::makeRigidTranslation(SurgSim::Math::Vector3d(0.0, 0.05, 0.0));

	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Sphere");
	element->setPose(pose);

	auto physics  = std::make_shared<RigidRepresentation>("Physics");
	physics->setDensity(5800.0);
	auto shape = std::make_shared<SurgSim::Math::SphereShape>(0.025);
	physics->setShape(shape);
	element->addComponent(physics);

	auto collision = std::make_shared<RigidCollisionRepresentation>("Collision");
	physics->setCollisionRepresentation(collision);
	element->addComponent(collision);

	auto graphics = std::make_shared<SurgSim::Graphics::OsgSphereRepresentation>("Graphics");
	graphics->setRadius(shape->getRadius());
	element->addComponent(graphics);

	return element;
}

TEST_F(RenderTests, SimulatedWoundRenderTest)
{
	runtime->getScene()->addSceneElement(createFemSceneElement("Fem",
										 "Geometry/wound_deformable.ply",
										 SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	runTest(Vector3d(0.0, 0.0, 0.2), Vector3d::Zero(), 5000.0);
}

TEST_F(RenderTests, Fem3dMeshCollision)
{
	runtime->getScene()->addSceneElement(createMeshSphere());

	runtime->getScene()->addSceneElement(createFemSceneElement("Fem",
										 "Geometry/wound_deformable.ply",
										 SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	runTest(Vector3d(0.0, 0.0, 0.2), Vector3d::Zero(), 5000.0);
}

} // namespace Physics
} // namespace SurgSim

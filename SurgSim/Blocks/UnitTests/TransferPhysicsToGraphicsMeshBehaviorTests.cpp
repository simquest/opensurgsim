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

/// \file
/// Tests for the TransferPhysicsToGraphicsBehavior class.

#include <gtest/gtest.h>

#include "SurgSim/Blocks/TransferPhysicsToGraphicsMeshBehavior.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"

using SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Runtime;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::OsgMeshRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::Fem3DRepresentation;
using SurgSim::Physics::RigidRepresentation;

namespace SurgSim
{
namespace Blocks
{

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, Constructor)
{
	ASSERT_NO_THROW(TransferPhysicsToGraphicsMeshBehavior("TestBehavior"));
}

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, SetGetSource)
{
	auto physics = std::make_shared<Fem3DRepresentation>("PhysicsDeformable");
	auto rigid = std::make_shared<RigidRepresentation>("PhysicsRigid");
	auto behavior = std::make_shared<TransferPhysicsToGraphicsMeshBehavior>("Behavior");

	EXPECT_THROW(behavior->setSource(nullptr), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(behavior->setSource(rigid), SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(behavior->setSource(physics));
	EXPECT_EQ(physics, behavior->getSource());
}

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, SetGetTarget)
{
	auto graphics = std::make_shared<OsgMeshRepresentation>("OsgMesh");
	auto graphicsBox = std::make_shared<OsgBoxRepresentation>("OsgBox");
	auto behavior = std::make_shared<TransferPhysicsToGraphicsMeshBehavior>("Behavior");

	EXPECT_THROW(behavior->setTarget(nullptr), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(behavior->setTarget(graphicsBox), SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(behavior->setTarget(graphics));
	EXPECT_EQ(graphics, behavior->getTarget());
}

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, Update)
{
	auto runtime = std::make_shared<Runtime>("config.txt");
	auto behaviorManager = std::make_shared<BehaviorManager>();
	runtime->addManager(behaviorManager);

	auto scene = runtime->getScene();
	auto sceneElement = std::make_shared<BasicSceneElement>("scene element");

	auto physics = std::make_shared<Fem3DRepresentation>("Fem3D");
	physics->loadFem("Geometry/wound_deformable.ply");

	auto graphics = std::make_shared<OsgMeshRepresentation>("GraphicsMesh");
	auto behavior = std::make_shared<TransferPhysicsToGraphicsMeshBehavior>("Behavior");
	behavior->setSource(physics);
	behavior->setTarget(graphics);

	sceneElement->addComponent(behavior);
	sceneElement->addComponent(physics);
	sceneElement->addComponent(graphics);
	scene->addSceneElement(sceneElement);

	// Test doInitialize(), doWakeUP()
	EXPECT_NO_THROW(runtime->start());
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	auto finalState = physics->getFinalState();
	auto numNodes = finalState->getNumNodes();
	auto target = graphics->getMesh();
	ASSERT_NE(0u, target->getNumVertices());
	ASSERT_NE(0u, numNodes);
	ASSERT_EQ(numNodes, target->getNumVertices());

	for (size_t nodeId = 0; nodeId < numNodes; ++nodeId)
	{
		EXPECT_TRUE(finalState->getPosition(nodeId).isApprox(target->getVertex(nodeId).position));
	}

	// Test TransferPhysicsToGraphicsBehavior::update()
	finalState->reset();
	behavior->update(1.0);

	for (size_t nodeId = 0; nodeId < numNodes; ++nodeId)
	{
		EXPECT_TRUE(target->getVertex(nodeId).position.isApprox(Vector3d::Zero()));
	}

	runtime->stop();
}

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, Serialization)
{
	std::string filename = std::string("Geometry/wound_deformable_with_texture.ply");

	std::shared_ptr<SurgSim::Framework::Component> physics = std::make_shared<Fem3DRepresentation>("Fem3D");
	auto fem3d = std::dynamic_pointer_cast<Fem3DRepresentation>(physics);
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	fem3d->loadFem(filename);

	std::shared_ptr<SurgSim::Framework::Component> graphics =
		std::make_shared<OsgMeshRepresentation>("GraphicsMesh");
	auto behavior = std::make_shared<TransferPhysicsToGraphicsMeshBehavior>("Behavior");

	EXPECT_NO_THROW(behavior->setValue("Source", physics));
	EXPECT_NO_THROW(behavior->setValue("Target", graphics));

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*behavior));
	EXPECT_EQ(1u, node.size());

	YAML::Node data = node["SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior"];
	std::shared_ptr<TransferPhysicsToGraphicsMeshBehavior> newBehavior;
	ASSERT_NO_THROW(newBehavior = std::dynamic_pointer_cast<TransferPhysicsToGraphicsMeshBehavior>(
									  node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

	EXPECT_EQ("SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior", newBehavior->getClassName());
	EXPECT_NE(nullptr, newBehavior->getValue<std::shared_ptr<SurgSim::Physics::DeformableRepresentation>>("Source"));
	EXPECT_NE(nullptr, newBehavior->getValue<std::shared_ptr<SurgSim::Graphics::MeshRepresentation>>("Target"));


}

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, MappingAccessors)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	auto behavior = std::make_shared<TransferPhysicsToGraphicsMeshBehavior>("Behavior");

	auto names = std::make_pair(std::string("TransferPhysicsToGraphicsMeshBehavior/data.ply"),
								std::string("TransferPhysicsToGraphicsMeshBehavior/data.ply"));

	// Plain Setter
	auto indices = behavior->getIndexMap();
	std::vector<std::pair<size_t, size_t>> empty;
	EXPECT_TRUE(indices.empty());

	indices.push_back(std::make_pair(0, 0));
	behavior->setValue("IndexMap", indices);
	EXPECT_EQ(1u, behavior->getIndexMap().size());

	// SetValue with pair
	ASSERT_NO_THROW(behavior->setValue("IndexMapMeshNames", names));
	indices = behavior->getValue<std::vector<std::pair<size_t, size_t>>>("IndexMap");
	EXPECT_EQ(4, indices.size());

	// SetValue with pair of meshes
	std::shared_ptr<Framework::Asset> mesh1 = std::make_shared<DataStructures::TriangleMeshPlain>();
	mesh1->load("TransferPhysicsToGraphicsMeshBehavior/data.ply");

	std::shared_ptr<Framework::Asset> mesh2 = std::make_shared<DataStructures::TriangleMeshPlain>();
	mesh2->load("TransferPhysicsToGraphicsMeshBehavior/data_more.ply");

	auto meshes = std::make_pair(mesh1, mesh2);
	ASSERT_NO_THROW(behavior->setValue("IndexMapMeshes", meshes));
	EXPECT_EQ(9u, behavior->getIndexMap().size());

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*behavior));
	EXPECT_EQ(1u, node.size());

	YAML::Node data = node["SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior"];
	std::shared_ptr<TransferPhysicsToGraphicsMeshBehavior> newBehavior;
	ASSERT_NO_THROW(newBehavior = std::dynamic_pointer_cast<TransferPhysicsToGraphicsMeshBehavior>(
									  node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

	EXPECT_EQ(9u, newBehavior->getIndexMap().size());
}

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, Mapping)
{
	Framework::ApplicationData data("config.txt");
	auto mesh1 = std::make_shared<DataStructures::TriangleMeshPlain>();
	mesh1->load("TransferPhysicsToGraphicsMeshBehavior/data.ply", data);

	auto indices = generateIndexMap(mesh1, mesh1);

	EXPECT_EQ(mesh1->getNumVertices(), indices.size());
	for (const auto& pair : indices)
	{
		EXPECT_EQ(pair.first, pair.second);
	}

	auto mesh2 = std::make_shared<DataStructures::TriangleMeshPlain>();
	mesh2->load("TransferPhysicsToGraphicsMeshBehavior/data_more.ply", data);

	indices = generateIndexMap(mesh1, mesh2);
	EXPECT_EQ(9u, indices.size());

	std::array<size_t, 4> counts = {0, 0, 0, 0};
	size_t total = 0;
	for (const auto& pair : indices)
	{
		++counts[pair.first];
		// Nothing should refer to the first vertex
		EXPECT_NE(0, pair.second);
	}

	EXPECT_EQ(0u, counts[0]);
	EXPECT_EQ(2u, counts[1]);
	EXPECT_EQ(3u, counts[2]);
	EXPECT_EQ(4u, counts[3]);
}

}
}


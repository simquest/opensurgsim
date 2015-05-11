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
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
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

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, ConstructorTest)
{
	ASSERT_NO_THROW(TransferPhysicsToGraphicsMeshBehavior("TestBehavior"));
}

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, SetGetSourceTest)
{
	auto physics = std::make_shared<Fem3DRepresentation>("PhysicsDeformable");
	auto rigid = std::make_shared<RigidRepresentation>("PhysicsRigid");
	auto behavior = std::make_shared<TransferPhysicsToGraphicsMeshBehavior>("Behavior");

	EXPECT_THROW(behavior->setSource(nullptr), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(behavior->setSource(rigid), SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(behavior->setSource(physics));
	EXPECT_EQ(physics, behavior->getSource());
}

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, SetGetTargetTest)
{
	auto graphics = std::make_shared<OsgMeshRepresentation>("OsgMesh");
	auto graphicsBox = std::make_shared<OsgBoxRepresentation>("OsgBox");
	auto behavior = std::make_shared<TransferPhysicsToGraphicsMeshBehavior>("Behavior");

	EXPECT_THROW(behavior->setTarget(nullptr), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(behavior->setTarget(graphicsBox), SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(behavior->setTarget(graphics));
	EXPECT_EQ(graphics, behavior->getTarget());
}

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, UpdateTest)
{
	auto runtime = std::make_shared<Runtime>("config.txt");
	auto behaviorManager = std::make_shared<BehaviorManager>();
	runtime->addManager(behaviorManager);

	auto scene = runtime->getScene();
	auto sceneElement = std::make_shared<BasicSceneElement>("scene element");

	auto physics = std::make_shared<Fem3DRepresentation>("Fem3D");
	physics->setFilename("Geometry/wound_deformable.ply");

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

TEST(TransferPhysicsToGraphicsMeshBehaviorTests, SerializationTest)
{
    std::string filename = std::string("../../SurgSim/Testing/Data/Geometry/wound_deformable.ply");

	std::shared_ptr<SurgSim::Framework::Component> physics = std::make_shared<Fem3DRepresentation>("Fem3D");
	auto fem3d = std::dynamic_pointer_cast<Fem3DRepresentation>(physics);
	fem3d->setFilename(filename);

	std::shared_ptr<SurgSim::Framework::Component> graphics =
		std::make_shared<OsgMeshRepresentation>("GraphicsMesh");
	auto behavior = std::make_shared<TransferPhysicsToGraphicsMeshBehavior>("Behavior");

	EXPECT_NO_THROW(behavior->setValue("Source", physics));
	EXPECT_NO_THROW(behavior->setValue("Target", graphics));

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*behavior));
	EXPECT_EQ(1u, node.size());

	YAML::Node data = node["SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior"];
	EXPECT_EQ(5u, data.size());

	std::shared_ptr<TransferPhysicsToGraphicsMeshBehavior> newBehavior;
	ASSERT_NO_THROW(newBehavior = std::dynamic_pointer_cast<TransferPhysicsToGraphicsMeshBehavior>(
									node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

	EXPECT_EQ("SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior", newBehavior->getClassName());
	EXPECT_NE(nullptr, newBehavior->getValue<std::shared_ptr<SurgSim::Physics::DeformableRepresentation>>("Source"));
	EXPECT_NE(nullptr, newBehavior->getValue<std::shared_ptr<SurgSim::Graphics::MeshRepresentation>>("Target"));
}

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
/// Tests for the VisualizeContactsBehavior class.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "SurgSim/Blocks/VisualizeContactsBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"

using SurgSim::Blocks::VisualizeContactsBehavior;
using SurgSim::Physics::RigidCollisionRepresentation;
using YAML::Node;

TEST(VisualizeContactsBehaviorTests, Constructor)
{
	EXPECT_NO_THROW(std::make_shared<VisualizeContactsBehavior>("VisualizeContactsBehavior"));
	EXPECT_NO_THROW({ VisualizeContactsBehavior visualizeContactsBehavior("VisualizeContactsBehavior"); });
}

TEST(VisualizeContactsBehaviorTests, SettersGetters)
{
	auto visualizeContactsBehavior = std::make_shared<VisualizeContactsBehavior>("VisualizeContactsBehavior");

	// Test collision representation.
	std::string name = "CollisionRepresentation";
	auto collisionRepresentaiton = std::make_shared<RigidCollisionRepresentation>(name);

	EXPECT_NO_THROW(visualizeContactsBehavior->setCollisionRepresentation(collisionRepresentaiton));
	EXPECT_EQ(name, visualizeContactsBehavior->getCollisionRepresentation()->getName());

	// Test vector field scale.
	EXPECT_ANY_THROW(visualizeContactsBehavior->setVectorFieldScale(-1.023));

	double scale = 1.234;
	EXPECT_NO_THROW(visualizeContactsBehavior->setVectorFieldScale(scale));
	EXPECT_EQ(scale, visualizeContactsBehavior->getVectorFieldScale());
}

TEST(VisualizeContactsBehaviorTests, Serialization)
{
	auto visualizeContactsBehavior = std::make_shared<VisualizeContactsBehavior>("VisualizeContactsBehavior");
	std::string name = "CollisionRepresentation";
	auto collisionRepresentation = std::make_shared<RigidCollisionRepresentation>(name);
	EXPECT_NO_THROW(visualizeContactsBehavior->setValue("CollisionRepresentation",
		std::static_pointer_cast<SurgSim::Framework::Component>(collisionRepresentation)););
	double scale = 1.234;
	EXPECT_NO_THROW(visualizeContactsBehavior->setValue("VectorFieldScale", scale));
	EXPECT_EQ("SurgSim::Blocks::VisualizeContactsBehavior", visualizeContactsBehavior->getClassName());

	// Encode
	Node node;
	EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*visualizeContactsBehavior));
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(5, node[visualizeContactsBehavior->getClassName()].size());

	// Decode
	std::shared_ptr<VisualizeContactsBehavior> newVisualizeContactsBehavior;
	EXPECT_NO_THROW(newVisualizeContactsBehavior = std::dynamic_pointer_cast<VisualizeContactsBehavior>(
		node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

	// Verify
	EXPECT_EQ(name, SurgSim::Framework::convert<std::shared_ptr<SurgSim::Framework::Component>>(
					newVisualizeContactsBehavior->getValue("CollisionRepresentation"))->getName());
	EXPECT_EQ(scale, SurgSim::Framework::convert<double>(newVisualizeContactsBehavior->getValue("VectorFieldScale")));
}

TEST(VisualizeContactsBehaviorTests, MultipleInstances)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto manager = std::make_shared<SurgSim::Graphics::OsgManager>();
	runtime->addManager(manager);

	auto visualizeContactsBehavior1 = std::make_shared<VisualizeContactsBehavior>("VisualizeContactsBehavior1");
	auto visualizeContactsBehavior2 = std::make_shared<VisualizeContactsBehavior>("VisualizeContactsBehavior2");
	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("SceneElement");
	sceneElement->addComponent(visualizeContactsBehavior1);
	sceneElement->addComponent(visualizeContactsBehavior2);

	auto collisionRepresentation = std::make_shared<RigidCollisionRepresentation>("CollisionRepresentation");

	EXPECT_NO_THROW(visualizeContactsBehavior1->setCollisionRepresentation(collisionRepresentation));
	EXPECT_NO_THROW(visualizeContactsBehavior2->setCollisionRepresentation(collisionRepresentation));

	EXPECT_TRUE(visualizeContactsBehavior1->initialize(runtime));
	EXPECT_TRUE(visualizeContactsBehavior2->initialize(runtime));

	EXPECT_TRUE(visualizeContactsBehavior1->wakeUp());
	EXPECT_TRUE(visualizeContactsBehavior2->wakeUp());
}
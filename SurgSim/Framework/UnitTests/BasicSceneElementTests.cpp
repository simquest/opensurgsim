// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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
/// Tests for the BasicSceneElement class.

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/UnitTests/MockObjects.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

#include "SurgSim/Testing/Utilities.h"

#include <gtest/gtest.h>

#include <random>

using SurgSim::Framework::Behavior;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

namespace
{
SURGSIM_REGISTER(SurgSim::Framework::Component, MockRepresentation, MockRepresentation);
}

namespace SurgSim
{

namespace Framework
{

TEST(BasicSceneElementTests, InitTest)
{
	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>("test name");

	EXPECT_EQ("test name", sceneElement->getName());
}

TEST(BasicSceneElementTests, InitComponentTest)
{
	std::shared_ptr<SurgSim::Framework::SceneElement> sceneElement = std::make_shared<BasicSceneElement>(
				"SceneElement");

	/// Scene element needs a runtime to initialize
	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>();
	sceneElement->setRuntime(runtime);

	std::shared_ptr<MockRepresentation> representation1 = std::make_shared<MockRepresentation>("TestRepresentation1");
	std::shared_ptr<MockRepresentation> representation2 = std::make_shared<MockRepresentation>("TestRepresentation2");

	sceneElement->addComponent(representation1);
	sceneElement->addComponent(representation2);

	EXPECT_FALSE(representation1->didInit());
	EXPECT_FALSE(representation1->didWakeUp());
	EXPECT_FALSE(representation2->didInit());
	EXPECT_FALSE(representation2->didWakeUp());

}

TEST(BasicSceneElementTests, SerializationTest)
{
	std::shared_ptr<SceneElement> sceneElement = std::make_shared<BasicSceneElement>("SceneElement");

	auto representation1 = std::make_shared<MockRepresentation>("TestRepresentation1");
	auto representation2 = std::make_shared<MockRepresentation>("TestRepresentation2");

	sceneElement->addComponent(representation1);
	sceneElement->addComponent(representation2);
	sceneElement->setActive(false);
	sceneElement->addToGroup("One");

	RigidTransform3d pose(makeRigidTransform(Quaterniond(0.0, 1.0, 0.0, 0.0), Vector3d(1.0, 2.0, 3.0)));
	sceneElement->setPose(pose);

	YAML::Node node;
	ASSERT_NO_THROW(sceneElement->encode(false)) << "Failed to serialize a SceneElement";;
	ASSERT_NO_THROW(node = sceneElement->encode(true)) << "Failed to serialize a SceneElement";

	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ("SurgSim::Framework::BasicSceneElement", node.begin()->first.as<std::string>());

	std::shared_ptr<SceneElement> result;

	ASSERT_NO_THROW(result = node.as<std::shared_ptr<SceneElement>>()) << "Failed to restore SceneElement.";
	EXPECT_EQ("SceneElement", result->getName());
	EXPECT_EQ(3u, result->getComponents().size());
	EXPECT_TRUE(pose.isApprox(result->getPose()));
	EXPECT_FALSE(result->isActive());
	EXPECT_TRUE(SurgSim::Testing::contains(result->getGroups(), "One"));
}

};  // namespace Blocks
};  // namespace SurgSim
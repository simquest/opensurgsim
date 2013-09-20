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
/// Tests for the TransferPoseBehavior class.

#include <SurgSim/Blocks/BasicSceneElement.h>
#include <SurgSim/Blocks/RepresentationPoseBehavior.h>
#include <SurgSim/Blocks/UnitTests/MockObjects.h>
#include <SurgSim/Framework/BehaviorManager.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Framework::Behavior;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

namespace SurgSim
{

namespace Blocks
{

TEST(TransferPoseBehaviorTests, InitTest)
{
	std::shared_ptr<MockRepresentation> from = std::make_shared<MockRepresentation>("from");
	std::shared_ptr<MockRepresentation> to = std::make_shared<MockRepresentation>("to");

	std::shared_ptr<Behavior> behavior = std::make_shared<TransferPoseBehavior>("test name", from, to);

	EXPECT_EQ("test name", behavior->getName());
}

TEST(TransferPoseBehaviorTests, UpdateTest)
{
	std::shared_ptr<MockRepresentation> from = std::make_shared<MockRepresentation>("from");
	std::shared_ptr<MockRepresentation> to = std::make_shared<MockRepresentation>("to");

	std::shared_ptr<Behavior> behavior = std::make_shared<TransferPoseBehavior>("behavior", from, to);

	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>();

	/// Add the representations and behavior to a scene element
	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>("scene element");
	sceneElement->addComponent(from);
	sceneElement->addComponent(to);
	sceneElement->addComponent(behavior);

	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager =
		std::make_shared<SurgSim::Framework::BehaviorManager>();

	runtime->addManager(behaviorManager);

	/// Create a scene and add the scene element to it
	std::shared_ptr<SurgSim::Framework::Scene> scene = std::make_shared<SurgSim::Framework::Scene>();
	scene->addSceneElement(sceneElement);
	runtime->setScene(scene);

	/// Set the initial pose of the "from" representation
	Quaterniond rotation = Quaterniond(SurgSim::Math::Vector4d::Random()).normalized();
	Vector3d position = Vector3d::Random();
	RigidTransform3d pose = makeRigidTransform(rotation, position);
	from->setInitialPose(pose);

	runtime->start();

	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	/// Check that initial pose propagates correctly
	EXPECT_TRUE(pose.matrix().isApprox(to->getPose().matrix())) <<
		"The behavior should copy the initial pose on update!";

	/// Change the pose and check that it propagates correctly
	rotation = Quaterniond(SurgSim::Math::Vector4d::Random()).normalized();
	position = Vector3d::Random();
	pose = makeRigidTransform(rotation, position);
	from->setPose(pose);

	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	EXPECT_TRUE(pose.matrix().isApprox(to->getPose().matrix())) <<
		"The behavior should copy the new pose on update!";

	runtime->stop();
}

};  // namespace Blocks
};  // namespace SurgSim

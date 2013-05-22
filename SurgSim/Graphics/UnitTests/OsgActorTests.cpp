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
/// Tests for the OsgActor class.

#include "SurgSim/Graphics/UnitTests/MockOsgObjects.h"

#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

#include "gtest/gtest.h"

#include <random>

using SurgSim::Graphics::Actor;
using SurgSim::Graphics::OsgActor;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

TEST(OsgActorTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Actor> actor = std::make_shared<MockOsgActor>("test name");});
}

TEST(OsgActorTests, OsgNodeTest)
{
	std::shared_ptr<OsgActor> actor = std::make_shared<MockOsgActor>("test name");

	EXPECT_NE(nullptr, actor->getOsgNode());

	/// Check that the OSG node is a group (MockOsgActor passes a new group as the node into the OsgActor constructor)
	osg::ref_ptr<osg::Group> osgGroup = dynamic_cast<osg::Group*>(actor->getOsgNode().get());
	EXPECT_TRUE(osgGroup.valid()) << "Actor's OSG node should be a group!";
}

TEST(OsgActorTests, NameTest)
{
	std::shared_ptr<Actor> actor = std::make_shared<MockOsgActor>("test name");

	EXPECT_EQ("test name", actor->getName());
}

TEST(OsgActorTests, VisibilityTest)
{
	std::shared_ptr<Actor> actor = std::make_shared<MockOsgActor>("test name");

	actor->setVisible(true);
	EXPECT_TRUE(actor->isVisible());

	actor->setVisible(false);
	EXPECT_FALSE(actor->isVisible());
}

TEST(OsgActorTests, PoseTest)
{
	std::shared_ptr<Actor> actor = std::make_shared<MockOsgActor>("test name");

	EXPECT_TRUE(actor->getPose().isApprox(RigidTransform3d::Identity()));

	/// Create a rigid body transform
	Vector3d translation = Vector3d::Random();
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 3.14);
	Quaterniond quaternion = SurgSim::Math::makeRotationQuaternion(distribution(generator), Vector3d(1.0, 0.0, 0.0));
	RigidTransform3d transform = SurgSim::Math::makeRigidTransform(quaternion, translation);

	/// Set the transform and make sure it was set correctly
	actor->setPose(transform);
	EXPECT_TRUE(actor->getPose().isApprox(transform));
}

TEST(OsgActorTests, UpdateTest)
{
	std::shared_ptr<MockOsgActor> mockActor = std::make_shared<MockOsgActor>("test name");
	std::shared_ptr<Actor> actor = mockActor;

	EXPECT_EQ(0, mockActor->getNumUpdates());
	EXPECT_EQ(0.0, mockActor->getSumDt());

	double sumDt = 0.0;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	/// Do 10 updates with random dt and check each time that the number of updates and sum of dt are correct.
	for (int i = 1; i <= 10; ++i)
	{
		double dt = distribution(generator);
		sumDt += dt;

		actor->update(dt);
		EXPECT_EQ(i, mockActor->getNumUpdates());
		EXPECT_LT(fabs(sumDt - mockActor->getSumDt()), Eigen::NumTraits<double>::dummy_precision());
	}
}

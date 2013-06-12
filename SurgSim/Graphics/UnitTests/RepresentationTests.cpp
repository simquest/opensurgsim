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
/// Tests for the Representation class.

#include <SurgSim/Graphics/UnitTests/MockObjects.h>

#include <SurgSim/Math/Quaternion.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Graphics::Representation;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

TEST(RepresentationTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Representation> representation = std::make_shared<MockRepresentation>("test name");});
}

TEST(RepresentationTests, NameTest)
{
	std::shared_ptr<Representation> representation = std::make_shared<MockRepresentation>("test name");

	EXPECT_EQ("test name", representation->getName());
}

TEST(RepresentationTests, VisibilityTest)
{
	std::shared_ptr<Representation> representation = std::make_shared<MockRepresentation>("test name");

	representation->setVisible(true);
	EXPECT_TRUE(representation->isVisible());

	representation->setVisible(false);
	EXPECT_FALSE(representation->isVisible());
}

TEST(RepresentationTests, PoseTest)
{
	std::shared_ptr<Representation> representation = std::make_shared<MockRepresentation>("test name");

	{
		SCOPED_TRACE("Check Initial Pose");
		EXPECT_TRUE(representation->getInitialPose().isApprox(RigidTransform3d::Identity()));
		EXPECT_TRUE(representation->getCurrentPose().isApprox(RigidTransform3d::Identity()));
		EXPECT_TRUE(representation->getFinalPose().isApprox(RigidTransform3d::Identity()));
	}

	RigidTransform3d initialPose;
	{
		SCOPED_TRACE("Set Initial Pose");
		initialPose = SurgSim::Math::makeRigidTransform(
			Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		representation->setInitialPose(initialPose);
		EXPECT_TRUE(representation->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(representation->getCurrentPose().isApprox(initialPose));
		EXPECT_TRUE(representation->getFinalPose().isApprox(initialPose));
	}

	{
		SCOPED_TRACE("Set Current Pose");
		RigidTransform3d currentPose = SurgSim::Math::makeRigidTransform(
			Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		representation->setCurrentPose(currentPose);
		EXPECT_TRUE(representation->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(representation->getCurrentPose().isApprox(currentPose));
		EXPECT_TRUE(representation->getFinalPose().isApprox(currentPose));
	}

	{
		SCOPED_TRACE("Change Initial Pose");
		initialPose = SurgSim::Math::makeRigidTransform(
			Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		representation->setInitialPose(initialPose);
		EXPECT_TRUE(representation->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(representation->getCurrentPose().isApprox(initialPose));
		EXPECT_TRUE(representation->getFinalPose().isApprox(initialPose));
	}
}

TEST(RepresentationTests, UpdateTest)
{
	std::shared_ptr<MockRepresentation> mockRepresentation = std::make_shared<MockRepresentation>("test name");
	std::shared_ptr<Representation> representation = mockRepresentation;

	EXPECT_EQ(0, mockRepresentation->getNumUpdates());
	EXPECT_EQ(0.0, mockRepresentation->getSumDt());

	double sumDt = 0.0;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	/// Do 10 updates with random dt and check each time that the number of updates and sum of dt are correct.
	for (int i = 1; i <= 10; ++i)
	{
		double dt = distribution(generator);
		sumDt += dt;

		representation->update(dt);
		EXPECT_EQ(i, mockRepresentation->getNumUpdates());
		EXPECT_LT(fabs(sumDt - mockRepresentation->getSumDt()), Eigen::NumTraits<double>::dummy_precision());
	}
}

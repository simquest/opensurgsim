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
/// Tests for the Camera class.

#include <SurgSim/Graphics/UnitTests/MockObjects.h>

#include <SurgSim/Math/Quaternion.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Graphics::Camera;
using SurgSim::Graphics::Group;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

TEST(CameraTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Camera> camera = std::make_shared<MockCamera>("test name");});
}

TEST(CameraTests, NameTest)
{
	std::shared_ptr<Camera> camera = std::make_shared<MockCamera>("test name");

	EXPECT_EQ("test name", camera->getName());
}

TEST(CameraTests, VisibilityTest)
{
	std::shared_ptr<Camera> camera = std::make_shared<MockCamera>("test name");

	camera->setVisible(true);
	EXPECT_TRUE(camera->isVisible());

	camera->setVisible(false);
	EXPECT_FALSE(camera->isVisible());
}

TEST(CameraTests, GroupTest)
{
	std::shared_ptr<Camera> camera = std::make_shared<MockCamera>("test name");

	std::shared_ptr<Group> group = std::make_shared<MockGroup>("test group");

	EXPECT_TRUE(camera->setGroup(group));

	EXPECT_EQ(group, camera->getGroup());
}

TEST(CameraTests, PoseTest)
{
	std::shared_ptr<Camera> camera = std::make_shared<MockCamera>("test name");

	{
		SCOPED_TRACE("Check Initial Pose");
		EXPECT_TRUE(camera->getInitialPose().isApprox(RigidTransform3d::Identity()));
		EXPECT_TRUE(camera->getCurrentPose().isApprox(RigidTransform3d::Identity()));
		EXPECT_TRUE(camera->getFinalPose().isApprox(RigidTransform3d::Identity()));
	}

	RigidTransform3d initialPose;
	{
		SCOPED_TRACE("Set Initial Pose");
		initialPose = SurgSim::Math::makeRigidTransform(
			Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		camera->setInitialPose(initialPose);
		EXPECT_TRUE(camera->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(camera->getCurrentPose().isApprox(initialPose));
		EXPECT_TRUE(camera->getFinalPose().isApprox(initialPose));
	}

	{
		SCOPED_TRACE("Set Current Pose");
		RigidTransform3d currentPose = SurgSim::Math::makeRigidTransform(
			Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		camera->setCurrentPose(currentPose);
		EXPECT_TRUE(camera->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(camera->getCurrentPose().isApprox(currentPose));
		EXPECT_TRUE(camera->getFinalPose().isApprox(currentPose));
	}

	{
		SCOPED_TRACE("Change Initial Pose");
		initialPose = SurgSim::Math::makeRigidTransform(
			Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		camera->setInitialPose(initialPose);
		EXPECT_TRUE(camera->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(camera->getCurrentPose().isApprox(initialPose));
		EXPECT_TRUE(camera->getFinalPose().isApprox(initialPose));
	}
}

TEST(CameraTests, MatricesTest)
{
	std::shared_ptr<Camera> camera = std::make_shared<MockCamera>("test name");

	/// Create a random view and projection matrix
	Matrix44d viewMatrix = Matrix44d::Random();
	Matrix44d projectionMatrix = Matrix44d::Random();

	/// Set the matrices and make sure they were set correctly
	camera->setViewMatrix(viewMatrix);
	EXPECT_TRUE(camera->getViewMatrix().isApprox(viewMatrix));

	camera->setProjectionMatrix(projectionMatrix);
	EXPECT_TRUE(camera->getProjectionMatrix().isApprox(projectionMatrix));
}

TEST(CameraTests, UpdateTest)
{
	std::shared_ptr<MockCamera> mockCamera = std::make_shared<MockCamera>("test name");
    std::shared_ptr<Camera> camera = mockCamera;

	EXPECT_EQ(0, mockCamera->getNumUpdates());
	EXPECT_EQ(0.0, mockCamera->getSumDt());

	double sumDt = 0.0;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	/// Do 10 updates with random dt and check each time that the number of updates and sum of dt are correct.
	for (int i = 1; i <= 10; ++i)
	{
		double dt = distribution(generator);
		sumDt += dt;

		camera->update(dt);
		EXPECT_EQ(i, mockCamera->getNumUpdates());
		EXPECT_LT(fabs(sumDt - mockCamera->getSumDt()), Eigen::NumTraits<double>::dummy_precision());
	}
}

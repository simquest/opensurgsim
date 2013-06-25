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
/// Tests for the OsgBoxRepresentation class.

#include <SurgSim/Graphics/UnitTests/MockOsgObjects.h>

#include <SurgSim/Graphics/OsgBoxRepresentation.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

namespace SurgSim
{

namespace Graphics
{

TEST(OsgBoxRepresentationTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Representation> representation =
		std::make_shared<OsgBoxRepresentation>("test name");});

	std::shared_ptr<Representation> representation = std::make_shared<OsgBoxRepresentation>("test name");
	EXPECT_EQ("test name", representation->getName());
}

TEST(OsgBoxRepresentationTests, VisibilityTest)
{
	std::shared_ptr<Representation> representation = std::make_shared<OsgBoxRepresentation>("test name");

	representation->setVisible(true);
	EXPECT_TRUE(representation->isVisible());

	representation->setVisible(false);
	EXPECT_FALSE(representation->isVisible());
}

TEST(OsgBoxRepresentationTests, SizeXTest)
{
	std::shared_ptr<BoxRepresentation> boxRepresentation = std::make_shared<OsgBoxRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	double randomSize = distribution(generator);

	boxRepresentation->setSizeX(randomSize);
	EXPECT_EQ(randomSize, boxRepresentation->getSizeX());
}

TEST(OsgBoxRepresentationTests, SizeYTest)
{
	std::shared_ptr<BoxRepresentation> boxRepresentation = std::make_shared<OsgBoxRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	double randomSize = distribution(generator);

	boxRepresentation->setSizeY(randomSize);
	EXPECT_EQ(randomSize, boxRepresentation->getSizeY());
}

TEST(OsgBoxRepresentationTests, SizeZTest)
{
	std::shared_ptr<BoxRepresentation> boxRepresentation = std::make_shared<OsgBoxRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	double randomSize = distribution(generator);

	boxRepresentation->setSizeZ(randomSize);
	EXPECT_EQ(randomSize, boxRepresentation->getSizeZ());
}

TEST(OsgBoxRepresentationTests, SizeTest)
{
	std::shared_ptr<BoxRepresentation> boxRepresentation = std::make_shared<OsgBoxRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	double randomSizeX = distribution(generator);
	double randomSizeY = distribution(generator);
	double randomSizeZ = distribution(generator);

	boxRepresentation->setSize(randomSizeX, randomSizeY, randomSizeZ);
	EXPECT_EQ(randomSizeX, boxRepresentation->getSizeX());
	EXPECT_EQ(randomSizeY, boxRepresentation->getSizeY());
	EXPECT_EQ(randomSizeZ, boxRepresentation->getSizeZ());
}

TEST(OsgBoxRepresentationTests, SizeVector3dTest)
{
	std::shared_ptr<BoxRepresentation> boxRepresentation = std::make_shared<OsgBoxRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	Vector3d randomSize(distribution(generator), distribution(generator), distribution(generator));

	boxRepresentation->setSize(randomSize);
	EXPECT_EQ(randomSize.x(), boxRepresentation->getSizeX());
	EXPECT_EQ(randomSize.y(), boxRepresentation->getSizeY());
	EXPECT_EQ(randomSize.z(), boxRepresentation->getSizeZ());
}

TEST(OsgBoxRepresentationTests, PoseTest)
{
	std::shared_ptr<Representation> representation = std::make_shared<MockOsgRepresentation>("test name");

	{
		SCOPED_TRACE("Check Initial Pose");
		EXPECT_TRUE(representation->getInitialPose().isApprox(RigidTransform3d::Identity()));
		EXPECT_TRUE(representation->getPose().isApprox(RigidTransform3d::Identity()));
	}

	RigidTransform3d initialPose;
	{
		SCOPED_TRACE("Set Initial Pose");
		initialPose = SurgSim::Math::makeRigidTransform(
			Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		representation->setInitialPose(initialPose);
		EXPECT_TRUE(representation->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(representation->getPose().isApprox(initialPose));
	}

	{
		SCOPED_TRACE("Set Current Pose");
		RigidTransform3d currentPose = SurgSim::Math::makeRigidTransform(
			Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		representation->setPose(currentPose);
		EXPECT_TRUE(representation->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(representation->getPose().isApprox(currentPose));
	}

	{
		SCOPED_TRACE("Change Initial Pose");
		initialPose = SurgSim::Math::makeRigidTransform(
			Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		representation->setInitialPose(initialPose);
		EXPECT_TRUE(representation->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(representation->getPose().isApprox(initialPose));
	}
}

};  // namespace Graphics

};  // namespace SurgSim

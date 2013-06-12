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
/// Tests for the OsgPlaneRepresentation class.

#include <SurgSim/Graphics/UnitTests/MockOsgObjects.h>

#include <SurgSim/Graphics/OsgPlaneRepresentation.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

namespace SurgSim
{

namespace Graphics
{

TEST(OsgPlaneRepresentationTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Representation> representation =
		std::make_shared<OsgPlaneRepresentation>("test name");});

	std::shared_ptr<Representation> representation = std::make_shared<OsgPlaneRepresentation>("test name");
	EXPECT_EQ("test name", representation->getName());
}

TEST(OsgPlaneRepresentationTests, VisibilityTest)
{
	std::shared_ptr<Representation> representation = std::make_shared<OsgPlaneRepresentation>("test name");

	representation->setVisible(true);
	EXPECT_TRUE(representation->isVisible());

	representation->setVisible(false);
	EXPECT_FALSE(representation->isVisible());
}

TEST(OsgPlaneRepresentationTests, PoseTest)
{
	std::shared_ptr<Representation> representation = std::make_shared<MockOsgRepresentation>("test name");

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

};  // namespace Graphics

};  // namespace SurgSim

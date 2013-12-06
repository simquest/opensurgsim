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
/// Tests for the OsgCamera class.

#include "SurgSim/Graphics/UnitTests/MockObjects.h"
#include "SurgSim/Graphics/UnitTests/MockOsgObjects.h"

#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgGroup.h"
#include "SurgSim/Graphics/OsgMatrixConversions.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Math/Quaternion.h"

#include <osg/Camera>
#include <osg/ref_ptr>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Math::Matrix44d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Graphics
{

TEST(OsgCameraTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Camera> camera = std::make_shared<OsgCamera>("test name");});

	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<OsgCamera>("test name");
	std::shared_ptr<Camera> camera = osgCamera;

	EXPECT_EQ("test name", camera->getName());

	EXPECT_TRUE(camera->isVisible());

	EXPECT_TRUE(camera->getPose().matrix().isApprox(
					fromOsg(osgCamera->getOsgCamera()->getViewMatrix()).inverse())) <<
							"Camera's pose should be initialized to the inverse of the osg::Camera's view matrix!";

	EXPECT_TRUE(camera->getViewMatrix().isApprox(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()))) <<
			"Camera's view matrix should be initialized to the osg::Camera's view matrix!";

	EXPECT_TRUE(camera->getProjectionMatrix().isApprox(fromOsg(osgCamera->getOsgCamera()->getProjectionMatrix()))) <<
			"Camera's projection matrix should be initialized to the osg::Camera's projection matrix!";

	EXPECT_NE(nullptr, camera->getGroup());
}

TEST(OsgCameraTests, OsgNodesTest)
{
	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<OsgCamera>("test name");
	std::shared_ptr<OsgRepresentation> osgRepresentation = osgCamera;

	/// Check that the OSG nodes of the camera are built correctly
	osg::ref_ptr<osg::Node> node = osgRepresentation->getOsgNode();
	osg::ref_ptr<osg::Switch> switchNode = dynamic_cast<osg::Switch*>(node.get());
	EXPECT_TRUE(switchNode.valid());
	EXPECT_EQ(1u, switchNode->getNumChildren());

	osg::ref_ptr<osg::Camera> camera = osgCamera->getOsgCamera();
	EXPECT_EQ(camera.get(), switchNode->getChild(0));
}

TEST(OsgCameraTests, VisibilityTest)
{
	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<OsgCamera>("test name");
	std::shared_ptr<OsgRepresentation> osgRepresentation = osgCamera;
	std::shared_ptr<Camera> camera = osgCamera;

	// Get the osg::Switch from the OsgRepresentation so that we can make sure that the osg::Camera has the
	// correct visibility.
	osg::ref_ptr<osg::Switch> switchNode = dynamic_cast<osg::Switch*>(osgRepresentation->getOsgNode().get());
	EXPECT_TRUE(switchNode.valid());

	EXPECT_TRUE(camera->isVisible());
	EXPECT_TRUE(switchNode->getChildValue(osgCamera->getOsgCamera()));

	camera->setVisible(false);
	EXPECT_FALSE(camera->isVisible());
	EXPECT_FALSE(switchNode->getChildValue(osgCamera->getOsgCamera()));

	camera->setVisible(true);
	EXPECT_TRUE(camera->isVisible());
	EXPECT_TRUE(switchNode->getChildValue(osgCamera->getOsgCamera()));

}

TEST(OsgCameraTests, GroupTest)
{
	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<OsgCamera>("test name");
	std::shared_ptr<Camera> camera = osgCamera;

	EXPECT_NE(nullptr, camera->getGroup());

	/// Adding an OsgGroup should succeed
	std::shared_ptr<OsgGroup> osgGroup = std::make_shared<OsgGroup>("test group");
	std::shared_ptr<Group> group = osgGroup;
	EXPECT_TRUE(camera->setGroup(group));
	EXPECT_EQ(group, camera->getGroup());

	/// Check that the OSG node of the group is added to the OSG camera correctly
	EXPECT_EQ(osgGroup->getOsgGroup(), osgCamera->getOsgCamera()->getChild(0)->asGroup()->getChild(0));

	/// Adding a group that does not derive from OsgGroup should fail
	std::shared_ptr<Group> mockGroup = std::make_shared<MockGroup>("non-osg group");
	EXPECT_FALSE(camera->setGroup(mockGroup));
	EXPECT_EQ(group, camera->getGroup());
	EXPECT_EQ(osgGroup->getOsgGroup(), osgCamera->getOsgCamera()->getChild(0)->asGroup()->getChild(0));
}


TEST(OsgCameraTests, PoseTest)
{
	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<OsgCamera>("test name");
	std::shared_ptr<Camera> camera = osgCamera;

	{
		SCOPED_TRACE("Check Initial Pose");
		EXPECT_TRUE(camera->getInitialPose().isApprox(RigidTransform3d::Identity()));
		EXPECT_TRUE(camera->getPose().isApprox(RigidTransform3d::Identity()));
	}

	RigidTransform3d initialPose;
	{
		SCOPED_TRACE("Set Initial Pose");
		initialPose = SurgSim::Math::makeRigidTransform(
						  Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		camera->setInitialPose(initialPose);
		EXPECT_TRUE(camera->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(camera->getPose().isApprox(initialPose));
		EXPECT_TRUE(camera->getViewMatrix().isApprox(initialPose.matrix().inverse()));
		EXPECT_TRUE(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()).isApprox(initialPose.matrix().inverse()));
	}

	{
		SCOPED_TRACE("Set Current Pose");
		RigidTransform3d currentPose = SurgSim::Math::makeRigidTransform(
			Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		camera->setPose(currentPose);
		EXPECT_TRUE(camera->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(camera->getPose().isApprox(currentPose));
		EXPECT_TRUE(camera->getViewMatrix().isApprox(currentPose.matrix().inverse()));
		EXPECT_TRUE(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()).isApprox(currentPose.matrix().inverse()));
	}

	{
		SCOPED_TRACE("Change Initial Pose");
		initialPose = SurgSim::Math::makeRigidTransform(
						  Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		camera->setInitialPose(initialPose);
		EXPECT_TRUE(camera->getInitialPose().isApprox(initialPose));
		EXPECT_TRUE(camera->getPose().isApprox(initialPose));
		EXPECT_TRUE(camera->getViewMatrix().isApprox(initialPose.matrix().inverse()));
		EXPECT_TRUE(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()).isApprox(initialPose.matrix().inverse()));
	}
}

TEST(OsgCameraTests, MatricesTest)
{
	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<OsgCamera>("test name");
	std::shared_ptr<Camera> camera = osgCamera;

	/// Create a random view and projection matrix
	RigidTransform3d viewTransform = SurgSim::Math::makeRigidTransform(
		Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
	Matrix44d viewMatrix = viewTransform.matrix();

	Matrix44d projectionMatrix = Matrix44d::Random();

	/// Set the matrices and make sure they were set correctly
	camera->setViewMatrix(viewMatrix);
	EXPECT_TRUE(camera->getViewMatrix().isApprox(viewMatrix));
	EXPECT_TRUE(camera->getPose().matrix().isApprox(viewMatrix.inverse()));
	EXPECT_TRUE(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()).isApprox(viewMatrix));

	camera->setProjectionMatrix(projectionMatrix);
	EXPECT_TRUE(camera->getProjectionMatrix().isApprox(projectionMatrix));
}

TEST(OsgCameraTests, RenderTargetTest)
{
	auto osgCamera = std::make_shared<OsgCamera>("test camera");
	std::shared_ptr<Camera> camera = osgCamera;

	std::shared_ptr<RenderTarget> renderTarget = std::make_shared<OsgRenderTarget2d>(256, 256, 1.0, 2, true);

	EXPECT_NO_THROW(camera->setRenderTarget(renderTarget));
	EXPECT_TRUE(osgCamera->getOsgCamera()->isRenderToTextureCamera());
}


}  // namespace Graphics
}  // namespace SurgSim

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

#include <SurgSim/Graphics/UnitTests/MockObjects.h>
#include <SurgSim/Graphics/UnitTests/MockOsgObjects.h>

#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgMatrixConversions.h>
#include <SurgSim/Math/Quaternion.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Graphics::Camera;
using SurgSim::Graphics::Group;
using SurgSim::Graphics::OsgActor;
using SurgSim::Graphics::OsgCamera;
using SurgSim::Graphics::OsgGroup;
using SurgSim::Graphics::fromOsg;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

TEST(OsgCameraTests, InitTest)
{
	ASSERT_NO_THROW({ std::shared_ptr<Camera> camera = std::make_shared<OsgCamera>("test name"); });

	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<OsgCamera>("test name");
	std::shared_ptr<Camera> camera = osgCamera;

	EXPECT_EQ("test name", camera->getName());

	EXPECT_TRUE(camera->isVisible());

	EXPECT_TRUE(camera->getPose().matrix().isApprox(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()).inverse())) <<
		"Camera's pose should be initialized to the inverse of the osg::Camera's view matrix!";

	EXPECT_TRUE(camera->getViewMatrix().isApprox(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()))) <<
		"Camera's view matrix should be initialized to the osg::Camera's view matrix!";

	EXPECT_TRUE(camera->getProjectionMatrix().isApprox(fromOsg(osgCamera->getOsgCamera()->getProjectionMatrix()))) <<
		"Camera's projection matrix should be initialized to the osg::Camera's projection matrix!";

	EXPECT_EQ(nullptr, camera->getGroup());
}

TEST(OsgCameraTests, OsgNodesTest)
{
	std::shared_ptr<MockOsgCamera> mockCamera = std::make_shared<MockOsgCamera>("test name");
	std::shared_ptr<OsgCamera> osgCamera = mockCamera;
	std::shared_ptr<OsgActor> osgActor = mockCamera;

	/// Check that the OSG nodes of the camera are built correctly
	osg::ref_ptr<osg::Node> node = osgActor->getOsgNode();
	osg::ref_ptr<osg::Switch> switchNode = dynamic_cast<osg::Switch*>(node.get());
	EXPECT_EQ(switchNode, mockCamera->getOsgSwitch());
	EXPECT_TRUE(switchNode.valid());
	EXPECT_EQ(1u, switchNode->getNumChildren());

	osg::ref_ptr<osg::Camera> camera = osgCamera->getOsgCamera();
	EXPECT_EQ(camera, switchNode->getChild(0));
}

TEST(OsgCameraTests, VisibilityTest)
{
	std::shared_ptr<MockOsgCamera> mockCamera = std::make_shared<MockOsgCamera>("test name");
	std::shared_ptr<Camera> camera = mockCamera;

	EXPECT_TRUE(camera->isVisible());

	camera->setVisible(false);
	EXPECT_FALSE(camera->isVisible());

	camera->setVisible(true);
	EXPECT_TRUE(camera->isVisible());
}

TEST(OsgCameraTests, GroupTest)
{
	std::shared_ptr<Camera> camera = std::make_shared<MockOsgCamera>("test name");

	EXPECT_EQ(nullptr, camera->getGroup());

	/// Adding an OsgGroup should succeed
	std::shared_ptr<Group> osgGroup = std::make_shared<OsgGroup>("test group");
	EXPECT_TRUE(camera->setGroup(osgGroup));
	EXPECT_EQ(osgGroup, camera->getGroup());

	/// Adding a group that does not derive from OsgGroup should fail
	std::shared_ptr<Group> mockGroup = std::make_shared<MockGroup>("non-osg group");
	EXPECT_FALSE(camera->setGroup(mockGroup));
	EXPECT_EQ(osgGroup, camera->getGroup());
}

TEST(OsgCameraTests, PoseAndMatricesTest)
{
	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<MockOsgCamera>("test name");
	std::shared_ptr<Camera> camera = osgCamera;

	EXPECT_TRUE(camera->getPose().isApprox(RigidTransform3d::Identity()));

	/// Create a random rigid body transform
	Vector3d translation = Vector3d::Random();
	Quaterniond quaternion = Quaterniond(SurgSim::Math::Vector4d::Random());
	quaternion.normalize();
	RigidTransform3d transform = SurgSim::Math::makeRigidTransform(quaternion, translation);

	/// Set the transform and make sure it was set correctly
	camera->setPose(transform);
	EXPECT_TRUE(camera->getPose().isApprox(transform));
	EXPECT_TRUE(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()).isApprox(transform.matrix().inverse()));

	/// Create a random view and projection matrix
	Matrix44d viewMatrix = Matrix44d::Random();
	Matrix44d projectionMatrix = Matrix44d::Random();

	/// Set the matrices and make sure they were set correctly
	camera->setViewMatrix(viewMatrix);
	EXPECT_TRUE(camera->getViewMatrix().isApprox(viewMatrix));
	EXPECT_TRUE(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()).isApprox(viewMatrix));

	camera->setProjectionMatrix(projectionMatrix);
	EXPECT_TRUE(camera->getProjectionMatrix().isApprox(projectionMatrix));
	EXPECT_TRUE(fromOsg(osgCamera->getOsgCamera()->getProjectionMatrix()).isApprox(projectionMatrix));
}

TEST(OsgCameraTests, UpdateTest)
{
	std::shared_ptr<MockOsgCamera> mockCamera = std::make_shared<MockOsgCamera>("test name");
	std::shared_ptr<Camera> camera = mockCamera;

	EXPECT_EQ(0u, mockCamera->getNumUpdates());
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

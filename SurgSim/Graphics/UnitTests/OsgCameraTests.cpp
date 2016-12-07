// This file is a part of the OpenSurgSim project.
// Copyright 2012-2016, SimQuest Solutions Inc.
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


#include <gtest/gtest.h>
#include <osg/Camera>
#include <osg/ref_ptr>
#include <random>

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgGroup.h"
#include "SurgSim/Graphics/OsgMatrixConversions.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/UnitTests/MockObjects.h"
#include "SurgSim/Graphics/UnitTests/MockOsgObjects.h"
#include "SurgSim/Math/Quaternion.h"

using SurgSim::Framework::BasicSceneElement;
using SurgSim::Math::makeRigidTransform;
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

	EXPECT_TRUE(camera->isActive());

	EXPECT_TRUE(camera->getPose().matrix().isApprox(
					fromOsg(osgCamera->getOsgCamera()->getViewMatrix()).inverse())) <<
							"Camera's pose should be initialized to the inverse of the osg::Camera's view matrix!";

	EXPECT_TRUE(camera->getViewMatrix().isApprox(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()))) <<
			"Camera's view matrix should be initialized to the osg::Camera's view matrix!";

	EXPECT_TRUE(camera->getProjectionMatrix().isApprox(fromOsg(osgCamera->getOsgCamera()->getProjectionMatrix()))) <<
			"Camera's projection matrix should be initialized to the osg::Camera's projection matrix!";

	EXPECT_EQ(0u, camera->getRenderGroups().size());
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

TEST(OsgCameraTests, ActivenessTest)
{
	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<OsgCamera>("test name");
	std::shared_ptr<OsgRepresentation> osgRepresentation = osgCamera;
	std::shared_ptr<Camera> camera = osgCamera;

	// Get the osg::Switch from the OsgRepresentation so that we can make sure that the osg::Camera has the
	// correct visibility.
	osg::ref_ptr<osg::Switch> switchNode = dynamic_cast<osg::Switch*>(osgRepresentation->getOsgNode().get());
	EXPECT_TRUE(switchNode.valid());

	EXPECT_TRUE(camera->isActive());
	EXPECT_TRUE(switchNode->getChildValue(osgCamera->getOsgCamera()));

	camera->setLocalActive(false);
	EXPECT_FALSE(camera->isActive());
	EXPECT_FALSE(switchNode->getChildValue(osgCamera->getOsgCamera()));

	camera->setLocalActive(true);
	EXPECT_TRUE(camera->isActive());
	EXPECT_TRUE(switchNode->getChildValue(osgCamera->getOsgCamera()));

}

TEST(OsgCameraTests, GroupTest)
{
	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<OsgCamera>("test name");
	std::shared_ptr<Camera> camera = osgCamera;
	camera->addRenderGroupReference("test group");

	EXPECT_EQ(0u, camera->getRenderGroups().size());

	/// Adding an OsgGroup should succeed
	std::shared_ptr<OsgGroup> osgGroup = std::make_shared<OsgGroup>(camera->getRenderGroupReferences().front());
	std::shared_ptr<Group> group = osgGroup;
	EXPECT_TRUE(camera->setRenderGroup(group));
	EXPECT_EQ(group, camera->getRenderGroups().front());

	/// Check that the OSG node of the group is added to the OSG camera correctly
	EXPECT_EQ(osgGroup->getOsgGroup(), osgCamera->getOsgCamera()->getChild(0)->asGroup()->getChild(0));

	/// Adding a group that does not derive from OsgGroup should fail
	std::shared_ptr<Group> mockGroup = std::make_shared<MockGroup>(camera->getRenderGroupReferences().front());
	EXPECT_FALSE(camera->setRenderGroup(mockGroup));
	EXPECT_EQ(group, camera->getRenderGroups().front());
	EXPECT_EQ(osgGroup->getOsgGroup(), osgCamera->getOsgCamera()->getChild(0)->asGroup()->getChild(0));
}


TEST(OsgCameraTests, PoseTest)
{
	auto runtime = std::make_shared<Framework::Runtime>();
	auto scene = runtime->getScene();
	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<OsgCamera>("test name");
	std::shared_ptr<Camera> camera = osgCamera;
	camera->setRenderGroupReference("Test");
	std::shared_ptr<BasicSceneElement> element = std::make_shared<BasicSceneElement>("element");
	element->addComponent(camera);
	scene->addSceneElement(element);
	camera->wakeUp();

	RigidTransform3d elementPose = SurgSim::Math::makeRigidTransform(
									   Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
	RigidTransform3d localPose = SurgSim::Math::makeRigidTransform(
									 Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
	RigidTransform3d pose = elementPose * localPose;

	{
		SCOPED_TRACE("Check Initial Pose");
		EXPECT_TRUE(camera->getLocalPose().isApprox(RigidTransform3d::Identity()));
		EXPECT_TRUE(camera->getPose().isApprox(RigidTransform3d::Identity()));
	}

	{
		SCOPED_TRACE("Set Local Pose");
		camera->setLocalPose(localPose);
		EXPECT_TRUE(camera->getLocalPose().isApprox(localPose));
		EXPECT_TRUE(camera->getPose().isApprox(localPose));
		EXPECT_TRUE(camera->getViewMatrix().isApprox(localPose.matrix().inverse()));
		EXPECT_TRUE(camera->getViewMatrix().inverse().isApprox(camera->getInverseViewMatrix()));

		camera->update(0.01);
		EXPECT_TRUE(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()).isApprox(localPose.matrix().inverse()));
	}

	{
		SCOPED_TRACE("Set Element Pose");
		element->setPose(elementPose);
		EXPECT_TRUE(camera->getLocalPose().isApprox(localPose));
		EXPECT_TRUE(camera->getPose().isApprox(pose));
		EXPECT_TRUE(camera->getViewMatrix().isApprox(pose.matrix().inverse()));
		EXPECT_TRUE(camera->getViewMatrix().inverse().isApprox(camera->getInverseViewMatrix()));

		camera->update(0.01);
		EXPECT_TRUE(fromOsg(osgCamera->getOsgCamera()->getViewMatrix()).isApprox(pose.matrix().inverse()));
	}
}

TEST(OsgCameraTests, MatricesTest)
{
	std::shared_ptr<OsgCamera> osgCamera = std::make_shared<OsgCamera>("test name");
	std::shared_ptr<Camera> camera = osgCamera;

	Matrix44d projectionMatrix = Matrix44d::Random();
	camera->setProjectionMatrix(projectionMatrix);
	EXPECT_TRUE(camera->getProjectionMatrix().isApprox(projectionMatrix));
	EXPECT_TRUE(camera->getInverseProjectionMatrix().isApprox(projectionMatrix.inverse()));
}

TEST(OsgCameraTests, RenderTargetTest)
{
	auto osgCamera = std::make_shared<OsgCamera>("test camera");
	std::shared_ptr<Camera> camera = osgCamera;

	std::shared_ptr<RenderTarget> renderTarget = std::make_shared<OsgRenderTarget2d>(256, 256, 1.0, 2, true);

	EXPECT_NO_THROW(camera->setRenderTarget(renderTarget));
	EXPECT_TRUE(osgCamera->getOsgCamera()->isRenderToTextureCamera());
}

TEST(OsgCameraTests, CameraGroupTest)
{
	std::shared_ptr<Camera> camera = std::make_shared<OsgCamera>("TestRepresentation");

	camera->clearGroupReferences();
	camera->addGroupReference("test1");
	camera->addGroupReference("test2");

	EXPECT_EQ(2u, camera->getGroupReferences().size());

	camera->setRenderGroupReference("otherTest");
	EXPECT_EQ(2u, camera->getGroupReferences().size());

	// Setting the render group of a camera to the group that it is in should remove the group
	// from the set
	camera->setRenderGroupReference("test1");
	EXPECT_EQ(1u, camera->getGroupReferences().size());

	// Should not be able to set group reference to the current render group
	EXPECT_FALSE(camera->addGroupReference("test1"));
	EXPECT_EQ(1u, camera->getGroupReferences().size());
}

TEST(OsgCameraTests, Serialization)
{
	std::shared_ptr<OsgCamera> camera = std::make_shared<OsgCamera>("TestOsgCamera");

	// Set values.
	SurgSim::Math::Matrix44d projection = SurgSim::Math::Matrix44d::Random();
	std::array<double, 2> viewport = { 1024, 768};
	camera->setValue("ProjectionMatrix", projection);
	camera->setValue("ViewportSize", viewport);
	camera->setValue("AmbientColor", SurgSim::Math::Vector4d(0.1, 0.2, 0.3, 0.4));
	camera->setViewport(10, 20, 30, 50);

	// Serialize.
	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*camera););

	// Deserialize.
	std::shared_ptr<Camera> newCamera;
	newCamera = std::dynamic_pointer_cast<OsgCamera>(node.as<std::shared_ptr<SurgSim::Framework::Component>>());
	EXPECT_NE(nullptr, newCamera);

	// Verify.
	EXPECT_TRUE(boost::any_cast<SurgSim::Math::Matrix44d>(camera->getValue("ProjectionMatrix")).isApprox(
					boost::any_cast<SurgSim::Math::Matrix44d>(newCamera->getValue("ProjectionMatrix"))));

	typedef std::array<double, 2> ParamType;
	EXPECT_TRUE(boost::any_cast<ParamType>(camera->getValue("ViewportSize")) ==
				boost::any_cast<ParamType>(newCamera->getValue("ViewportSize")));

	EXPECT_TRUE(boost::any_cast<SurgSim::Math::Vector4d>(camera->getValue("AmbientColor")).isApprox(
					boost::any_cast<SurgSim::Math::Vector4d>(newCamera->getValue("AmbientColor"))));
}

TEST(OsgCameraTests, SetProjection)
{
	std::shared_ptr<OsgCamera> camera = std::make_shared<OsgCamera>("TestOsgCamera");

	Math::Matrix44d identity = Math::Matrix44d::Identity();

	camera->setProjectionMatrix(identity);
	osg::ref_ptr<osg::Camera> osgCamera(new osg::Camera);

	// Windows does not support initializer lists ...
	std::array<double, 4> persp = {{90.0, 1.0, 0.01, 10.0}};

	osgCamera->setProjectionMatrixAsPerspective(persp[0], persp[1], persp[2], persp[3]);
	auto expectedPerspective = Graphics::fromOsg(osgCamera->getProjectionMatrix());

	camera->setPerspectiveProjection(persp[0], persp[1], persp[2], persp[3]);
	EXPECT_TRUE(expectedPerspective.isApprox(camera->getProjectionMatrix()));

	camera->setProjectionMatrix(identity);

	EXPECT_NO_THROW(camera->setValue("PerspectiveProjection", persp));
	EXPECT_TRUE(expectedPerspective.isApprox(camera->getProjectionMatrix()));

	camera->setProjectionMatrix(identity);

	std::array<double, 6> ortho = {{ -1.0, 1.0, 2.0, -2.0, 3.0, -3.0}};

	osgCamera->setProjectionMatrixAsOrtho(ortho[0], ortho[1], ortho[2], ortho[3], ortho[4], ortho[5]);
	auto expectedOrtho = Graphics::fromOsg(osgCamera->getProjectionMatrix());

	camera->setOrthogonalProjection(ortho[0], ortho[1], ortho[2], ortho[3], ortho[4], ortho[5]);
	EXPECT_TRUE(expectedOrtho.isApprox(camera->getProjectionMatrix()));

	camera->setProjectionMatrix(identity);

	EXPECT_NO_THROW(camera->setValue("OrthogonalProjection", ortho));
	EXPECT_TRUE(expectedOrtho.isApprox(camera->getProjectionMatrix()));
}

TEST(OsgCameraTests, Viewport)
{
	std::shared_ptr<OsgCamera> camera = std::make_shared<OsgCamera>("TestOsgCamera");

	std::array<int, 4> original = {10, 20, 30, 40};
	std::array<int, 4> result = {0, 0, 0, 0};

	camera->setViewport(original[0], original[1], original[2], original[3]);
	camera->getViewport(&result[0], &result[1], &result[2], &result[3]);

	EXPECT_EQ(original, result);
	camera->setViewport(0, 0, 0, 0);

	camera->setValue("Viewport", original);
	result = camera->getValue<std::array<int, 4>>("Viewport");
	EXPECT_EQ(original, result);
}

TEST(OsgCameraTests, MultipleRenderGroups)
{
	std::shared_ptr<OsgCamera> camera = std::make_shared<OsgCamera>("TestOsgCamera");

	EXPECT_EQ(0u, camera->getRenderGroupReferences().size());

	camera->addRenderGroupReference("Group1");
	camera->addRenderGroupReference("Group2");
	EXPECT_EQ(2u, camera->getRenderGroupReferences().size());

	std::vector<std::string> references;
	references.push_back("Group3");
	references.push_back("Group4");
	references.push_back("Group5");
	camera->setRenderGroupReferences(references);
	EXPECT_EQ(3u, camera->getRenderGroupReferences().size());

	camera->addRenderGroup(std::make_shared<Graphics::OsgGroup>("Group3"));
	camera->addRenderGroup(std::make_shared<Graphics::OsgGroup>("Group4"));
	camera->addRenderGroup(std::make_shared<Graphics::OsgGroup>("Group5"));

	std::vector<std::shared_ptr<Group>> groups;
	camera->getOsgCamera()->removeChildren(0, 1);
	references.push_back("Group6");
	references.push_back("Group7");
	references.push_back("Group8");
	for (auto reference : references)
	{
		camera->addRenderGroupReference(reference);
		groups.push_back(std::make_shared<Graphics::OsgGroup>(reference));
	}
	camera->setRenderGroups(groups);
	EXPECT_EQ(groups.size(), camera->getRenderGroups().size());
}

}  // namespace Graphics
}  // namespace SurgSim

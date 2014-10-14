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
/// Tests for the OsgSphereRepresentation class.

#include "SurgSim/Graphics/UnitTests/MockOsgObjects.h"

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Framework/FrameworkConvert.h"

#include <gtest/gtest.h>

#include <osg/Geode>

#include <random>

using SurgSim::Framework::BasicSceneElement;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

namespace SurgSim
{

namespace Graphics
{

TEST(OsgSphereRepresentationTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Representation> representation =
						 std::make_shared<OsgSphereRepresentation>("test name");
					});

	std::shared_ptr<Representation> representation = std::make_shared<OsgSphereRepresentation>("test name");
	EXPECT_EQ("test name", representation->getName());
}

TEST(OsgSphereRepresentationTests, AccessibleTest)
{
	std::shared_ptr<SurgSim::Framework::Component> component;
	ASSERT_NO_THROW(component = SurgSim::Framework::Component::getFactory().create(
									"SurgSim::Graphics::OsgSphereRepresentation",
									"sphere"));

	EXPECT_EQ("SurgSim::Graphics::OsgSphereRepresentation", component->getClassName());

	double radius = 4.321;

	component->setValue("Radius", radius);
	YAML::Node node(YAML::convert<SurgSim::Framework::Component>::encode(*component));

	auto decoded = std::dynamic_pointer_cast<SurgSim::Graphics::OsgSphereRepresentation>(
					   node.as<std::shared_ptr<SurgSim::Framework::Component>>());

	EXPECT_NE(nullptr, decoded);
	EXPECT_DOUBLE_EQ(radius, decoded->getValue<double>("Radius"));
}

TEST(OsgSphereRepresentationTests, OsgNodeTest)
{
	std::shared_ptr<OsgRepresentation> representation = std::make_shared<OsgSphereRepresentation>("test name");

	ASSERT_NE(nullptr, representation->getOsgNode());

	osg::Switch* switchNode = dynamic_cast<osg::Switch*>(representation->getOsgNode().get());
	ASSERT_NE(nullptr, switchNode) << "Could not get OSG switch node!";

	ASSERT_EQ(1u, switchNode->getNumChildren()) << "OSG switch node should have 1 child, the transform node!";

	osg::PositionAttitudeTransform* transformNode =
		dynamic_cast<osg::PositionAttitudeTransform*>(switchNode->getChild(0));
	ASSERT_NE(nullptr, transformNode) << "Could not get OSG transform node!";

	ASSERT_EQ(1u, transformNode->getNumChildren()) << "OSG transform node should have 1 child, the geode!";

	osg::Node* node = dynamic_cast<osg::Node*>(transformNode->getChild(0));
	ASSERT_NE(nullptr, node) << "Could not get unit sphere OSG node!";
}

TEST(OsgSphereRepresentationTests, RadiusTest)
{
	std::shared_ptr<SphereRepresentation> sphereRepresentation = std::make_shared<OsgSphereRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	double randomRadius = distribution(generator);

	sphereRepresentation->setRadius(randomRadius);
	EXPECT_EQ(randomRadius, sphereRepresentation->getRadius());
}

TEST(OsgSphereRepresentationTests, PoseTest)
{
	std::shared_ptr<Representation> representation = std::make_shared<MockOsgRepresentation>("test name");
	std::shared_ptr<BasicSceneElement> element = std::make_shared<BasicSceneElement>("element");
	element->addComponent(representation);
	element->initialize();
	representation->wakeUp();

	{
		SCOPED_TRACE("Check Initial Pose");
		EXPECT_TRUE(representation->getLocalPose().isApprox(RigidTransform3d::Identity()));
		EXPECT_TRUE(representation->getPose().isApprox(RigidTransform3d::Identity()));
	}

	RigidTransform3d localPose;
	{
		SCOPED_TRACE("Set Local Pose");
		localPose = SurgSim::Math::makeRigidTransform(
						Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		representation->setLocalPose(localPose);
		EXPECT_TRUE(representation->getLocalPose().isApprox(localPose));
		EXPECT_TRUE(representation->getPose().isApprox(localPose));
	}

	RigidTransform3d elementPose;
	{
		SCOPED_TRACE("Set Element Pose");
		elementPose = SurgSim::Math::makeRigidTransform(
						  Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		element->setPose(elementPose);
		EXPECT_TRUE(representation->getLocalPose().isApprox(localPose));
		EXPECT_TRUE(representation->getPose().isApprox(elementPose * localPose));
	}

	{
		SCOPED_TRACE("Change Local Pose");
		localPose = SurgSim::Math::makeRigidTransform(
						Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(), Vector3d::Random());
		representation->setLocalPose(localPose);
		EXPECT_TRUE(representation->getLocalPose().isApprox(localPose));
		EXPECT_TRUE(representation->getPose().isApprox(elementPose * localPose));
	}
}

TEST(OsgSphereRepresentationTests, MaterialTest)
{
	std::shared_ptr<OsgRepresentation> osgRepresentation = std::make_shared<OsgSphereRepresentation>("test name");
	std::shared_ptr<Representation> representation = osgRepresentation;

	std::shared_ptr<OsgMaterial> osgMaterial = std::make_shared<OsgMaterial>("material");
	std::shared_ptr<Material> material = osgMaterial;
	{
		SCOPED_TRACE("Set material");
		EXPECT_TRUE(representation->setMaterial(material));
		EXPECT_EQ(material, representation->getMaterial());

		osg::Switch* switchNode = dynamic_cast<osg::Switch*>(osgRepresentation->getOsgNode().get());
		ASSERT_NE(nullptr, switchNode) << "Could not get OSG switch node!";
		ASSERT_EQ(1u, switchNode->getNumChildren()) << "OSG switch node should have 1 child, the transform node!";
		EXPECT_EQ(osgMaterial->getOsgStateSet(), switchNode->getChild(0)->getStateSet()) <<
				"State set should be the material's state set!";
	}

	{
		SCOPED_TRACE("Clear material");
		representation->clearMaterial();
		EXPECT_EQ(nullptr, representation->getMaterial());

		osg::Switch* switchNode = dynamic_cast<osg::Switch*>(osgRepresentation->getOsgNode().get());
		ASSERT_NE(nullptr, switchNode) << "Could not get OSG switch node!";
		ASSERT_EQ(1u, switchNode->getNumChildren()) << "OSG switch node should have 1 child, the transform node!";
		EXPECT_NE(osgMaterial->getOsgStateSet(), switchNode->getChild(0)->getStateSet()) <<
				"State set should have been cleared!";
	}
}

};  // namespace Graphics

};  // namespace SurgSim

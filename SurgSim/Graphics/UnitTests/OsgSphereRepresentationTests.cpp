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

};  // namespace Graphics

};  // namespace SurgSim

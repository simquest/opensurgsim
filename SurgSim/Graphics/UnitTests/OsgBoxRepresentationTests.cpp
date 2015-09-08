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

#include "SurgSim/Graphics/UnitTests/MockOsgObjects.h"

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Framework/FrameworkConvert.h"

#include <gtest/gtest.h>

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

TEST(OsgBoxRepresentationTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Representation> representation =
						 std::make_shared<OsgBoxRepresentation>("test name");
					});

	std::shared_ptr<Representation> representation = std::make_shared<OsgBoxRepresentation>("test name");
	EXPECT_EQ("test name", representation->getName());
}

TEST(OsgBoxRepresentationTests, AccessibleTest)
{
	std::shared_ptr<SurgSim::Framework::Component> component;
	ASSERT_NO_THROW(component = SurgSim::Framework::Component::getFactory().create(
									"SurgSim::Graphics::OsgBoxRepresentation",
									"box"));

	EXPECT_EQ("SurgSim::Graphics::OsgBoxRepresentation", component->getClassName());

	SurgSim::Math::Vector3d size(1.0, 2.0, 3.0);

	component->setValue("Size", size);
	YAML::Node node(YAML::convert<SurgSim::Framework::Component>::encode(*component));

	auto decoded = std::dynamic_pointer_cast<SurgSim::Graphics::OsgBoxRepresentation>(
					   node.as<std::shared_ptr<SurgSim::Framework::Component>>());

	EXPECT_NE(nullptr, decoded);
	EXPECT_TRUE(size.isApprox(decoded->getValue<SurgSim::Math::Vector3d>("Size")));
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

	boxRepresentation->setSizeXYZ(randomSizeX, randomSizeY, randomSizeZ);
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

};  // namespace Graphics

};  // namespace SurgSim

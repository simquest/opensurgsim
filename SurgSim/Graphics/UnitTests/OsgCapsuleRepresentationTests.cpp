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
/// Tests for the OsgCapsuleRepresentation class.

#include <SurgSim/Graphics/UnitTests/MockOsgObjects.h>

#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgCapsuleRepresentation.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

namespace SurgSim
{

namespace Graphics
{

TEST(OsgCapsuleRepresentationTests, RadiusTest)
{
	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	double randomSize = distribution(generator);

	capsuleRepresentation->setRadius(randomSize);
	EXPECT_EQ(randomSize, capsuleRepresentation->getRadius());
}

TEST(OsgCapsuleRepresentationTests, HeightTest)
{
	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	double randomSize = distribution(generator);

	capsuleRepresentation->setHeight(randomSize);
	EXPECT_EQ(randomSize, capsuleRepresentation->getHeight());
}

TEST(OsgCapsuleRepresentationTests, SizeTest)
{
	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	double randomRadius = distribution(generator);
	double randomHeight = distribution(generator);

	capsuleRepresentation->setSize(randomRadius, randomHeight);
	EXPECT_EQ(randomRadius, capsuleRepresentation->getRadius());
	EXPECT_EQ(randomHeight, capsuleRepresentation->getHeight());
}

TEST(OsgCapsuleRepresentationTests, SizeVectordTest)
{
	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	Vector2d randomSize(distribution(generator), distribution(generator));

	capsuleRepresentation->setSize(randomSize);
	EXPECT_EQ(randomSize, capsuleRepresentation->getSize());
}

};  // namespace Graphics

};  // namespace SurgSim

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
/// Tests for the OsgCylinderRepresentation class.

#include <SurgSim/Graphics/UnitTests/MockOsgObjects.h>

#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgCylinderRepresentation.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{

namespace Graphics
{

TEST(OsgCylinderRepresentationTests, RadiusTest)
{
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation =
		std::make_shared<OsgCylinderRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	double randomSize = distribution(generator);

	cylinderRepresentation->setRadius(randomSize);
	EXPECT_NEAR(randomSize, cylinderRepresentation->getRadius(), epsilon);
}

TEST(OsgCylinderRepresentationTests, HeightTest)
{
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation =
		std::make_shared<OsgCylinderRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	double randomSize = distribution(generator);

	cylinderRepresentation->setHeight(randomSize);
	EXPECT_NEAR(randomSize, cylinderRepresentation->getHeight(), epsilon);
}

TEST(OsgCylinderRepresentationTests, SizeTest)
{
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation =
		std::make_shared<OsgCylinderRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	double randomRadius = distribution(generator);
	double randomHeight = distribution(generator);

	cylinderRepresentation->setSize(randomRadius, randomHeight);
	EXPECT_NEAR(randomRadius, cylinderRepresentation->getRadius(), epsilon);
	EXPECT_NEAR(randomHeight, cylinderRepresentation->getHeight(), epsilon);
}

TEST(OsgCylinderRepresentationTests, SizeVector2dTest)
{
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation =
		std::make_shared<OsgCylinderRepresentation>("test name");

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(1.0, 10.0);

	Vector2d randomSize(distribution(generator), distribution(generator));

	cylinderRepresentation->setSize(randomSize);
	EXPECT_TRUE(randomSize.isApprox(cylinderRepresentation->getSize(), epsilon));
}


};  // namespace Graphics

};  // namespace SurgSim

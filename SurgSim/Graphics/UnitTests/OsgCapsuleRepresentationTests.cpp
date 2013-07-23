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

#include <SurgSim/Graphics/OsgCapsuleRepresentation.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Testing/MathUtilities.h>

#include <gtest/gtest.h>

using SurgSim::Math::Vector2d;
using SurgSim::Testing::interpolate;

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{

namespace Graphics
{

TEST(OsgCapsuleRepresentationTests, RadiusTest)
{
	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("test name");

	double startRadius = 1.0;
	double endRadius = 10.0;
	double interpolatedRadius;
	for (double t = 0; t < 100; ++t)
	{
		interpolatedRadius = interpolate(startRadius, endRadius, t/100);
		capsuleRepresentation->setRadius(interpolatedRadius);
		EXPECT_NEAR(capsuleRepresentation->getRadius(), interpolatedRadius, epsilon);
	}
}

TEST(OsgCapsuleRepresentationTests, HeightTest)
{
	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("test name");

	double startHeight = 1.0;
	double endHeight = 10.0;
	double interpolatedHeight;
	for (double t = 0; t < 100; ++t)
	{
		interpolatedHeight = interpolate(startHeight, endHeight, t/100);
		capsuleRepresentation->setHeight(interpolatedHeight);
		EXPECT_NEAR(capsuleRepresentation->getHeight(), interpolatedHeight, epsilon);
	}
}

TEST(OsgCapsuleRepresentationTests, SizeTest)
{
	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("test name");

	double startRadius = 1.0;
	double endRadius = 10.0;
	double startHeight = 1.0;
	double endHeight = 10.0;
	double interpolatedRadius, interpolatedHeight;
	double radius, height;
	for (double t = 0; t < 100; ++t)
	{
		interpolatedRadius = interpolate(startRadius, endRadius, t/100);
		interpolatedHeight = interpolate(startHeight, endHeight, t/100);
		capsuleRepresentation->setSize(interpolatedRadius, interpolatedHeight);
		capsuleRepresentation->getSize(&radius, &height);
		EXPECT_NEAR(radius, interpolatedRadius, epsilon);
		EXPECT_NEAR(height, interpolatedHeight, epsilon);
	}
}

TEST(OsgCapsuleRepresentationTests, SizeVectordTest)
{
	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("test name");

	double startRadius = 1.0;
	double endRadius = 10.0;
	double startHeight = 1.0;
	double endHeight = 10.0;
	Vector2d interpolation;
	for (double t = 0; t < 100; ++t)
	{
		interpolation.x() = interpolate(startRadius, endRadius, t/100);
		interpolation.y() = interpolate(startHeight, endHeight, t/100);
		capsuleRepresentation->setSize(interpolation);
		EXPECT_TRUE(capsuleRepresentation->getSize().isApprox(interpolation));
	}
}

};  // namespace Graphics

};  // namespace SurgSim

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

#include "SurgSim/Graphics/UnitTests/MockOsgObjects.h"

#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgCylinderRepresentation.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Framework/FrameworkConvert.h"

#include <gtest/gtest.h>

#include <random>

using SurgSim::Math::Vector2d;

namespace
{
const double epsilon = 1e-10;
};

namespace SurgSim
{

namespace Graphics
{

TEST(OsgCylinderRepresentationTests, AccessibleTest)
{
	std::shared_ptr<SurgSim::Framework::Component> component;
	ASSERT_NO_THROW(component = SurgSim::Framework::Component::getFactory().create(
									"SurgSim::Graphics::OsgCylinderRepresentation",
									"sphere"));

	EXPECT_EQ("SurgSim::Graphics::OsgCylinderRepresentation", component->getClassName());

	double radius = 4.321;
	double height = 1.234;

	component->setValue("Height", height);
	component->setValue("Radius", radius);

	YAML::Node node(YAML::convert<SurgSim::Framework::Component>::encode(*component));

	auto decoded = std::dynamic_pointer_cast<SurgSim::Graphics::OsgCylinderRepresentation>(
					   node.as<std::shared_ptr<SurgSim::Framework::Component>>());

	EXPECT_NE(nullptr, decoded);
	EXPECT_DOUBLE_EQ(radius, decoded->getValue<double>("Radius"));
	EXPECT_DOUBLE_EQ(height, decoded->getValue<double>("Height"));
}

TEST(OsgCylinderRepresentationTests, RadiusTest)
{
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation =
		std::make_shared<OsgCylinderRepresentation>("test name");

	double radius = 1.0;

	cylinderRepresentation->setRadius(radius);
	EXPECT_NEAR(radius, cylinderRepresentation->getRadius(), epsilon);
}

TEST(OsgCylinderRepresentationTests, HeightTest)
{
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation =
		std::make_shared<OsgCylinderRepresentation>("test name");

	double height = 1.0;

	cylinderRepresentation->setHeight(height);
	EXPECT_NEAR(height, cylinderRepresentation->getHeight(), epsilon);
}

TEST(OsgCylinderRepresentationTests, SizeTest)
{
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation =
		std::make_shared<OsgCylinderRepresentation>("test name");

	double radius = 1.0, height = 1.0;

	cylinderRepresentation->setSize(radius, height);
	EXPECT_NEAR(radius, cylinderRepresentation->getRadius(), epsilon);
	EXPECT_NEAR(height, cylinderRepresentation->getHeight(), epsilon);
}

TEST(OsgCylinderRepresentationTests, SizeVector2dTest)
{
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation =
		std::make_shared<OsgCylinderRepresentation>("test name");

	Vector2d size(1.0, 1.0);

	cylinderRepresentation->setSize(size);
	EXPECT_TRUE(size.isApprox(cylinderRepresentation->getSize(), epsilon));
}


};  // namespace Graphics

};  // namespace SurgSim

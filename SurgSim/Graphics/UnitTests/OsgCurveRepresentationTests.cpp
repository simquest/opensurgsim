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

#include "SurgSim/Graphics/OsgCurveRepresentation.h"
#include "SurgSim/DataStructures/Vertices.h"

#include <gtest/gtest.h>

namespace
{
const double epsilon = 1e-5;
}
namespace SurgSim
{

namespace Graphics
{

TEST(OsgCurveRepresentationTests, Init)
{
	ASSERT_NO_THROW({std::shared_ptr<Representation> representation =
						 std::make_shared<OsgCurveRepresentation>("Test");
					});
}

// Tests setters and getters through the properties, thereby also testing the properties
TEST(OsgCurveRepresentationTests, SetterGettersProperties)
{
	auto curve = std::make_shared<OsgCurveRepresentation>("Test");

	double width = 1234.0;
	curve->setValue("Width", width);
	EXPECT_NEAR(width, curve->getValue<double>("Width"), epsilon);
	EXPECT_NEAR(width, curve->getWidth(), epsilon);

	double tension = 0.1234;
	curve->setValue("Tension", tension);
	EXPECT_NEAR(tension, curve->getValue<double>("Tension"), epsilon);
	EXPECT_NEAR(tension, curve->getTension(), epsilon);

	bool val = curve->isAntiAliasing();
	curve->setValue("AntiAliasing", !val);
	EXPECT_EQ(!val, curve->isAntiAliasing());
	EXPECT_EQ(!val, curve->getValue<bool>("AntiAliasing"));

	size_t subdivisions = 4321;
	curve->setValue("Subdivisions", subdivisions);
	EXPECT_EQ(subdivisions, curve->getValue<size_t>("Subdivisions"));
	EXPECT_EQ(subdivisions, curve->getSubdivisions());

	Math::Vector4d color(4.0, 3.0, 2.0, 1.0);
	curve->setValue("Color", color);
	EXPECT_TRUE(color.isApprox(curve->getValue<Math::Vector4d>("Color")));
	EXPECT_TRUE(color.isApprox(curve->getColor()));
}

TEST(OsgCurveRepresentationTests, ControlPoints)
{
	typedef DataStructures::VerticesPlain::VertexType Vertex;
	DataStructures::VerticesPlain vertices;
	vertices.addVertex(Vertex(Math::Vector3d(1.0, 2.0, 3.0)));

	auto curve = std::make_shared<OsgCurveRepresentation>("Test");
	EXPECT_ANY_THROW(curve->updateControlPoints(vertices));

	vertices.addVertex(Vertex(Math::Vector3d(1.0, 2.0, 3.0)));
	vertices.addVertex(Vertex(Math::Vector3d(1.0, 2.0, 3.0)));

	EXPECT_NO_THROW(curve->updateControlPoints(vertices));

	EXPECT_NO_THROW(curve->setValue("Vertices", vertices));
}

}
}
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
/// Unit Tests for the OsgVectorFieldRepresentation class.

#include <SurgSim/Graphics/VectorFieldRepresentation.h>
#include <SurgSim/Graphics/OsgVectorFieldRepresentation.h>
#include <SurgSim/Math/Vector.h>

#include <gtest/gtest.h>

namespace
{
	const double epsilon = 1e-10;
};

using SurgSim::DataStructures::Vertex;
using SurgSim::DataStructures::Vertices;
using SurgSim::Graphics::OsgVectorFieldRepresentation;
using SurgSim::Graphics::VectorFieldRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;

TEST(OsgVectorFieldRepresentationTests, VerticesTest)
{
	// Locations in 3D space to draw vectors
	std::vector<Vector3d> points(8);
	points[0] = Vector3d(1.0, 0.0, 0.0);
	points[1] = Vector3d(0.0, 1.0, 0.0);
	points[2] = Vector3d(-1.0, 0.0, 0.0);
	points[3] = Vector3d(0.0, -1.0, 0.0);

	points[4] = Vector3d(2.0, 0.0, 0.0);
	points[5] = Vector3d(0.0, 2.0, 0.0);
	points[6] = Vector3d(-2.0, 0.0, 0.0);
	points[7] = Vector3d(0.0, -2.0, 0.0);

	std::vector<Vector4d> colors(8);
	colors[0] = Vector4d(1.0, 0.0, 0.0, 1.0);
	colors[1] = Vector4d(1.0, 0.0, 0.0, 1.0);
	colors[2] = Vector4d(1.0, 0.0, 0.0, 1.0);
	colors[3] = Vector4d(1.0, 0.0, 0.0, 1.0);

	colors[4] = Vector4d(1.0, 0.0, 0.0, 1.0);
	colors[5] = Vector4d(0.0, 1.0, 1.0, 1.0);
	colors[6] = Vector4d(1.0, 0.0, 0.0, 1.0);
	colors[7] = Vector4d(1.0, 0.0, 0.0, 1.0);

	// Vector3d: Coordinates of the (mathematical) vector
	// Vector4d: Color (R,G,B,alpha) information of the vector
	std::vector<SurgSim::Graphics::VectorFieldData> vectors(8);
	for (auto i = 0; i < 8; ++i)
	{
		vectors[i].vectorDirection.setValue(points[i]);
		vectors[i].vectorColor.setValue(colors[i]);
	}

	// Associate vectors to points (locations in 3D space)
	auto vertices = std::make_shared<SurgSim::Graphics::VectorField>();
	auto it = std::begin(points);
	auto v = std::begin(vectors);
	for (; it != std::end(points); ++it, ++v)
	{
		vertices->addVertex(Vertex<SurgSim::Graphics::VectorFieldData>((*it), *v));
	}

	std::shared_ptr<VectorFieldRepresentation> vectorFieldRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("Vector Field");
	vectorFieldRepresentation->setVectorField(vertices);
	EXPECT_EQ(vertices, vectorFieldRepresentation->getVectorField());
}

TEST(OsgVectorFieldRepresentationTests, LineWidthTest)
{
	std::shared_ptr<VectorFieldRepresentation> vectorFieldRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("Vector Field");
	vectorFieldRepresentation->setLineWidth(1.25);
	EXPECT_NEAR(1.25, vectorFieldRepresentation->getLineWidth(), epsilon);
}
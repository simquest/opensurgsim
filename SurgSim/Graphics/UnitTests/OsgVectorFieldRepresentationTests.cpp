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
	std::vector<Vector3d> points;
	points.emplace_back(Vector3d(1.0, 0.0, 0.0));
	points.emplace_back(Vector3d(0.0, 1.0, 0.0));
	points.emplace_back(Vector3d(-1.0, 0.0, 0.0));
	points.emplace_back(Vector3d(0.0, -1.0, 0.0));

	points.emplace_back(Vector3d(2.0, 0.0, 0.0));
	points.emplace_back(Vector3d(0.0, 2.0, 0.0));
	points.emplace_back(Vector3d(-2.0, 0.0, 0.0));
	points.emplace_back(Vector3d(0.0, -2.0, 0.0));

	// Vector3d: Coordinates of the (mathematical vector)
	// Vector4d: Color (R,G,B,alpha) information of the vector
	// Color information is optional. Color of vector is set to white by default
	std::vector<SurgSim::Graphics::VectorFieldData> vectors;
	vectors.emplace_back(Vector3d(1.0, 0.0, 0.0));
	vectors.emplace_back(Vector3d(0.0, 1.0, 0.0));
	vectors.emplace_back(Vector3d(-1.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 1.0));
	vectors.emplace_back(Vector3d(0.0, -1.0, 0.0));

	vectors.emplace_back(Vector3d(2.0, 0.0, 0.0));
	vectors.emplace_back(Vector3d(0.0, 2.0, 0.0), Vector4d(0.0, 1.0, 1.0, 1.0));
	vectors.emplace_back(Vector3d(-2.0, 0.0, 0.0));
	vectors.emplace_back(Vector3d(0.0, -2.0, 0.0));

	auto vertices = std::make_shared<SurgSim::Graphics::VectorField>();
	auto it = std::begin(points);
	auto v = std::begin(vectors);
	for (; it != std::end(points); ++it, ++v)
	{
		vertices->addVertex(Vertex<SurgSim::Graphics::VectorFieldData>((*it), *v));
	}

	std::shared_ptr<VectorFieldRepresentation> vectorFieldRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("Vector Field");
	vectorFieldRepresentation->setVertices(vertices);
	EXPECT_EQ(vertices, vectorFieldRepresentation->getVertices());
}

TEST(OsgVectorFieldRepresentationTests, LineWidthTest)
{
	std::shared_ptr<VectorFieldRepresentation> vectorFieldRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("Vector Field");
	vectorFieldRepresentation->setLineWidth(1.25);
	EXPECT_NEAR( 1.25, vectorFieldRepresentation->getLineWidth(), epsilon);
}
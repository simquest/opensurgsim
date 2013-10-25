// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

/// Render Tests for the OsgVectorFieldRepresentation class.

#include <SurgSim/DataStructures/Vertex.h>
#include <SurgSim/DataStructures/Vertices.h>
#include <SurgSim/Graphics/RenderTests/RenderTest.h>
#include <SurgSim/Graphics/VectorField.h>
#include <SurgSim/Graphics/VectorFieldRepresentation.h>
#include <SurgSim/Graphics/OsgVectorFieldRepresentation.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Testing/MathUtilities.h>

using SurgSim::DataStructures::Vertex;
using SurgSim::DataStructures::Vertices;
using SurgSim::Graphics::OsgVectorFieldRepresentation;
using SurgSim::Graphics::VectorFieldRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Testing::interpolate;

struct OsgVectorFieldRepresentationRenderTests : public SurgSim::Graphics::RenderTest
{
protected:
	std::vector<Vector3d> makePoints()
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
		return points;
	}

	std::vector<SurgSim::Graphics::VectorFieldData> makeVector()
	{
		std::vector<SurgSim::Graphics::VectorFieldData> points;
		// Vector3d: Coordinates of the (mathematical vector)
		// Vector4d: Color (R,G,B,alpha) information of the vector
		points.emplace_back(Vector3d(1.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
		points.emplace_back(Vector3d(0.0, 1.0, 0.0), Vector4d(0.0, 1.0, 0.0, 0.0));

		points.emplace_back(Vector3d(-1.0, 0.0, 0.0), Vector4d(0.0, 0.0, 1.0, 0.0));
		points.emplace_back(Vector3d(0.0, -1.0, 0.0), Vector4d(1.0, 1.0, 0.0, 0.0));

		points.emplace_back(Vector3d(2.0, 0.0, 0.0), Vector4d(1.0, 0.0, 1.0, 0.0));
		points.emplace_back(Vector3d(0.0, 2.0, 0.0), Vector4d(1.0, 1.0, 1.0, 0.0));

		points.emplace_back(Vector3d(-2.0, 0.0, 0.0));
		points.emplace_back(Vector3d(0.0, -2.0, 0.0));
		return points;
	}

	std::shared_ptr<VectorFieldRepresentation> makeVectorFieldRepresentation(const std::vector<Vector3d>& points)
	{
		auto vectorRepresentation =
			std::make_shared<OsgVectorFieldRepresentation>("vector filed representation");

		auto vertices = std::make_shared<SurgSim::Graphics::VectorField>();
		std::vector<SurgSim::Graphics::VectorFieldData> vectors = makeVector();

		// Binding vectors to points
		auto it = std::begin(points);
		auto v = std::begin(vectors);
		for (; it != std::end(points); ++it, ++v)
		{
			vertices->addVertex(Vertex<SurgSim::Graphics::VectorFieldData>((*it), *v));
		}

		vectorRepresentation->setVertices(vertices);
		vectorRepresentation->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, -8.0)));

		viewElement->addComponent(vectorRepresentation);
		return vectorRepresentation;
	}
};

TEST_F(OsgVectorFieldRepresentationRenderTests, LineWidth)
{
	std::shared_ptr<VectorFieldRepresentation> vectorRepresentation = makeVectorFieldRepresentation(makePoints());

	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	int numSteps = 100;
	double startWidth = 0.0;
	double endWidth = 10.0;

	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		vectorRepresentation->setLineWidth(interpolate(startWidth, endWidth, t));
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}
}
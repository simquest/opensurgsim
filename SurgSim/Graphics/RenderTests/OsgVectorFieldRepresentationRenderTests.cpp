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
	// A point is a location (X,Y,Z) in 3D space
	std::vector<Vector3d> makePoints()
	{
		std::vector<Vector3d> points(8);
		points[0] = Vector3d(1.0, 0.0, 0.0);
		points[1] = Vector3d(0.0, 1.0, 0.0);
		points[2] = Vector3d(-1.0, 0.0, 0.0);
		points[3] = Vector3d(0.0, -1.0, 0.0);

		points[4] = Vector3d(2.0, 0.0, 0.0);
		points[5] = Vector3d(0.0, 2.0, 0.0);
		points[6] = Vector3d(-2.0, 0.0, 0.0);
		points[7] = Vector3d(0.0, -2.0, 0.0);
		return points;
	}

	std::vector<Vector3d> makePoints2()
	{
		std::vector<Vector3d> points(8);
		points[0] = Vector3d(1.0, 1.0, 0.0);
		points[1] = Vector3d(-1.0, 1.0, 0.0);
		points[2] = Vector3d(-1.0, -1.0, 0.0);
		points[3] = Vector3d(1.0, -1.0, 0.0);

		points[4] = Vector3d(2.0, 2.0, 0.0);
		points[5] = Vector3d(-2.0, 2.0, 0.0);
		points[6] = Vector3d(-2.0, -2.0, 0.0);
		points[7] = Vector3d(2.0, -2.0, 0.0);
		return points;
	}

	// Color is represented as (R, G, B, alpha)
	std::vector<Vector4d> makeColors()
	{
		std::vector<Vector4d> colors(8);
		colors[0] = Vector4d(1.0, 0.0, 0.0, 0.0);
		colors[1] = Vector4d(0.0, 1.0, 0.0, 0.0);
		colors[2] = Vector4d(0.0, 0.0, 1.0, 0.0);
		colors[3] = Vector4d(1.0, 1.0, 0.0, 0.0);

		colors[4] = Vector4d(1.0, 0.0, 1.0, 0.0);
		colors[5] = Vector4d(1.0, 1.0, 1.0, 0.0);
		colors[6] = Vector4d(1.0, 0.5, 0.8, 0.0);
		colors[7] = Vector4d(0.5, 1.0, 0.5, 0.0);
		return colors;
	}

	std::vector<Vector4d> makeColors2()
	{
		std::vector<Vector4d> colors(8);
		colors[0] = Vector4d(0.0, 1.0, 0.0, 0.0);
		colors[1] = Vector4d(0.0, 0.0, 1.0, 0.0);
		colors[2] = Vector4d(1.0, 0.0, 0.0, 0.0);
		colors[3] = Vector4d(0.0, 1.0, 1.0, 0.0);

		colors[4] = Vector4d(0.0, 1.0, 0.0, 0.0);
		colors[5] = Vector4d(0.0, 0.0, 1.0, 0.0);
		colors[6] = Vector4d(1.0, 1.0, 0.0, 0.0);
		colors[7] = Vector4d(1.0, 0.0, 1.0, 0.0);
		return colors;
	}

	std::vector<SurgSim::Graphics::VectorFieldData> makeVectors(const std::vector<Vector3d>& points,
															   const std::vector<Vector4d>& colors)
	{
		std::vector<SurgSim::Graphics::VectorFieldData> vecs(8);
		for (auto i = 0; i < 8; ++i)
		{
			vecs[i].vectorDirection.setValue(points[i]);
			vecs[i].vectorColor.setValue(colors[i]);
		}
		return vecs;
	}

	std::shared_ptr< SurgSim::Graphics::VectorField>
		makeVectorField(const std::vector<Vector3d>& points,
						const std::vector<SurgSim::Graphics::VectorFieldData>& vectors)
	{
		auto vertices = std::make_shared<SurgSim::Graphics::VectorField>();

		// Binding vectors to points
		auto it = std::begin(points);
		auto v = std::begin(vectors);
		for (; it != std::end(points); ++it, ++v)
		{
			vertices->addVertex(Vertex<SurgSim::Graphics::VectorFieldData>((*it), *v));
		}
		return vertices;
	}
};

TEST_F(OsgVectorFieldRepresentationRenderTests, LineWidth)
{
	/* points: locations to draw vectors
	   vectors: A list of vectors associated with optional color information
	   colors: A list of colors (SurgSim::Math::Vector4d)
	   vertices: A Vector Field object
	*/
	auto points = makePoints();
	auto colors = makeColors();
	auto vectors = makeVectors(points, colors);
	auto vertices = makeVectorField(points, vectors);

	auto vectorRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("vector field representation");
	vectorRepresentation->setVectorField(vertices);
	vectorRepresentation->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, -8.0)));

	viewElement->addComponent(vectorRepresentation);
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	int numSteps = 100;
	double startWidth = 0.0;
	double endWidth = 10.0;

	// Vary line widths as time changes
	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;
		vectorRepresentation->setLineWidth(interpolate(startWidth, endWidth, t));
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}
}

TEST_F(OsgVectorFieldRepresentationRenderTests, ChangingLocations)
{
	auto startPoints = makePoints();
	auto endPoints = makePoints2();
	auto colors = makeColors();
	auto vectors = makeVectors(startPoints, colors);
	auto vertices = makeVectorField(startPoints, vectors);

	auto vectorRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("vector field representation");

	vectorRepresentation->setVectorField(vertices);
	vectorRepresentation->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, -8.0)));

	viewElement->addComponent(vectorRepresentation);
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	std::vector<Vector3d> points(8);
	int numSteps = 100;
	// Vary locations to draw the vector as time changes
	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;
		for (int j = 0; j < 8; ++j)
		{
			points[j] = interpolate(startPoints[j], endPoints[j], t);
		}
		auto newVertices = makeVectorField(points, vectors);
		vectorRepresentation->setVectorField(newVertices);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}
}

TEST_F(OsgVectorFieldRepresentationRenderTests, ChangingVectors)
{
	auto points = makePoints();
	auto startVectors= makePoints();
	auto endVectors = makePoints2();
	auto colors = makeColors();
	auto vectors = makeVectors(startVectors, colors);
	auto vertices = makeVectorField(points, vectors);

	auto vectorRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("vector field representation");
	vectorRepresentation->setVectorField(vertices);
	vectorRepresentation->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, -8.0)));

	viewElement->addComponent(vectorRepresentation);
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	std::vector<Vector3d> vecs(8);
	int numSteps = 100;
	// Vary vector directions as time changes
	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;
		for (int j = 0; j < 8; ++j)
		{
			vecs[j] = interpolate(startVectors[j], endVectors[j], t);
		}
		auto newVectors = makeVectors(vecs, colors);
		auto newVertices = makeVectorField(points, newVectors);
		vectorRepresentation->setVectorField(newVertices);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}
}

TEST_F(OsgVectorFieldRepresentationRenderTests, ChangingColors)
{
	auto points = makePoints();
	auto startColors= makeColors();
	auto endColors= makeColors2();
	auto vectors = makeVectors(points, startColors);
	auto vertices = makeVectorField(points, vectors);

	auto vectorRepresentation =
		std::make_shared<OsgVectorFieldRepresentation>("vector field representation");
	vectorRepresentation->setVectorField(vertices);
	vectorRepresentation->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, -8.0)));

	viewElement->addComponent(vectorRepresentation);
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	std::vector<Vector4d> colors(8);
	int numSteps = 100;
	// Vary colors as time changes
	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;
		for (int j = 0; j < 8; ++j)
		{
			colors[j] = interpolate(startColors[j], endColors[j], t);
		}
		auto newVectors = makeVectors(points, colors);
		auto newVertices = makeVectorField(points, newVectors);
		vectorRepresentation->setVectorField(newVertices);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}
}
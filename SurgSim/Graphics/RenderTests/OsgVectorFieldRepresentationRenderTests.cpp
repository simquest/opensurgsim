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

#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Graphics/VectorField.h"
#include "SurgSim/Graphics/VectorFieldRepresentation.h"
#include "SurgSim/Graphics/OsgVectorFieldRepresentation.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Testing/MathUtilities.h"

using SurgSim::DataStructures::Vertex;
using SurgSim::Graphics::OsgVectorFieldRepresentation;
using SurgSim::Graphics::VectorFieldRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Testing::interpolate;

typedef std::vector<SurgSim::Graphics::VectorFieldData, Eigen::aligned_allocator<SurgSim::Graphics::VectorFieldData>> VectorFieldDataVector;

struct OsgVectorFieldRepresentationRenderTests : public SurgSim::Graphics::RenderTest
{
protected:
	// A point is a location (X,Y,Z) in 3D space
	std::vector<Vector3d> makeStartingPoints()
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

	std::vector<Vector3d> makeEndingPoints()
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
	std::vector<Vector4d> makeStartingColors()
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

	std::vector<Vector4d> makeEndingColors()
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

	VectorFieldDataVector makeVectors(const std::vector<Vector3d>& points,
			const std::vector<Vector4d>& colors)
	{
		VectorFieldDataVector vecs(8);
		for (auto i = 0; i < 8; ++i)
		{
			vecs[i].direction = points[i];
			vecs[i].color.setValue(colors[i]);
		}
		return vecs;
	}

	std::shared_ptr<VectorFieldRepresentation>
	makeVectorFieldRepresentation(const std::vector<Vector3d>& points,
								  const VectorFieldDataVector& vectors)
	{
		auto representation = std::make_shared<OsgVectorFieldRepresentation>("Vector Field Representation");
		auto vertices =  representation->getVectorField();
		// Binding vectors to points
		auto it = std::begin(points);
		auto v = std::begin(vectors);
		for (; it != std::end(points); ++it, ++v)
		{
			vertices->addVertex(Vertex<SurgSim::Graphics::VectorFieldData>((*it), *v));
		}
		return representation;
	}
};

TEST_F(OsgVectorFieldRepresentationRenderTests, AddVectors)
{
	auto vectorRepresentation = std::make_shared<OsgVectorFieldRepresentation>("VectorField");
	auto points = makeStartingPoints();
	auto colors = makeStartingColors();
	auto vectors = makeVectors(points, colors);

	SurgSim::Graphics::VectorField vectorField;

	vectorRepresentation->setLocalPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, -8.0)));
	viewElement->addComponent(vectorRepresentation);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));

	auto it = std::begin(points);
	auto v = std::begin(vectors);
	for (; it != std::end(points); ++it, ++v)
	{
		vectorField.addVertex(Vertex<SurgSim::Graphics::VectorFieldData>((*it), *v));
		vectorRepresentation->updateVectorField(vectorField);
		boost::this_thread::sleep(boost::posix_time::milliseconds(250));
	}
}

TEST_F(OsgVectorFieldRepresentationRenderTests, LineWidth)
{
	auto points = makeStartingPoints();
	auto colors = makeStartingColors();
	auto vectors = makeVectors(points, colors);
	auto vectorRepresentation = makeVectorFieldRepresentation(points, vectors);

	vectorRepresentation->setLocalPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, -8.0)));

	viewElement->addComponent(vectorRepresentation);
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));

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

TEST_F(OsgVectorFieldRepresentationRenderTests, ChangingVectorField)
{
	auto startPoints = makeStartingPoints();
	auto endPoints = makeEndingPoints();

	auto startColors = makeStartingColors();
	auto endColors = makeStartingColors();

	auto startVectors = makeVectors(startPoints, startColors);
	auto endVectors = makeVectors(endPoints, endColors);

	auto vectorRepresentation = makeVectorFieldRepresentation(startPoints, endVectors);
	vectorRepresentation->setLocalPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, -8.0)));

	viewElement->addComponent(vectorRepresentation);
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(250));

	auto& vertexList = vectorRepresentation->getVectorField()->getVertices();
	int numSteps = 100;
	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;
		for (int j = 0; j < 8; ++j)
		{
			vertexList[j].position = interpolate(startPoints[j], endPoints[j], t);
			vertexList[j].data.direction = interpolate(endVectors[j].direction, startVectors[j].direction, t);
			vertexList[j].data.color.setValue(interpolate(startColors[j], endColors[j], t));
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}
}
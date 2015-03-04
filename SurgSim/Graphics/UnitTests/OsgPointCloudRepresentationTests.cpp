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

#include <vector>

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::EmptyData;
using SurgSim::Graphics::PointCloudRepresentation;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;

namespace
{
const double epsilon = 1e-5;
}

namespace SurgSim
{
namespace Graphics
{

TEST(OsgPointCloudRepresentationTests, InitTest)
{
	ASSERT_NO_THROW(OsgPointCloudRepresentation("TestPointCloud"));
};

TEST(OsgPointCloudRepresentationTests, PointSizeTest)
{
	auto pointCloud = std::make_shared<OsgPointCloudRepresentation>("TestPointCloud");

	double pointSize = 1.234;
	pointCloud->setPointSize(pointSize);
	EXPECT_NEAR(pointSize, pointCloud->getPointSize(), epsilon);
}

TEST(OsgPointCloudRepresentationTests, ColorTest)
{
	auto pointCloud = std::make_shared<OsgPointCloudRepresentation>("TestPointCloud");

	Vector4d color = Vector4d(1.0, 2.0, 3.0, 4.0);
	pointCloud->setColor(color);
	EXPECT_TRUE(color.isApprox(pointCloud->getColor()));
}

TEST(OsgPointCloudRepresentationTests, VertexTest)
{
	auto pointCloud = std::make_shared<OsgPointCloudRepresentation>("TestPointCloud");
	auto vertices = pointCloud->getVertices();
	EXPECT_EQ(0u, vertices->getNumVertices());

	std::vector<Vector3d> vertexList;
	vertexList.push_back(Vector3d(0.01, -0.01, 0.01));
	vertexList.push_back(Vector3d(0.01, -0.01, 0.01));
	vertexList.push_back(Vector3d(-0.01, -0.01, 0.01));
	vertexList.push_back(Vector3d(-0.01, -0.01, -0.01));
	vertexList.push_back(Vector3d(0.01, -0.01, -0.01));

	for (auto it = std::begin(vertexList); it != std::end(vertexList); ++it)
	{
		vertices->addVertex(SurgSim::Graphics::PointCloud::VertexType(*it));
	}

	auto updatedVertices = pointCloud->getVertices();
	ASSERT_EQ(vertexList.size(), updatedVertices->getNumVertices());
	for (size_t i = 0; i < vertexList.size(); ++i)
	{
		EXPECT_TRUE(vertexList[i].isApprox(updatedVertices->getVertexPosition(i)));
	}
}

TEST(OsgPointCloudRepresentationTests, SerializationTest)
{
	auto pointCloud = std::make_shared<OsgPointCloudRepresentation>("TestPointCloud");

	double pointSize = 1.234;
	Vector4d color = Vector4d(1.0, 2.0, 3.0, 4.0);

	pointCloud->setValue("PointSize", pointSize);
	pointCloud->setValue("Color", color);

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*pointCloud));

	EXPECT_EQ(1u, node.size());
	YAML::Node data;
	data = node["SurgSim::Graphics::OsgPointCloudRepresentation"];

	std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation> newOsgPointCloud;
	ASSERT_NO_THROW(newOsgPointCloud = std::dynamic_pointer_cast<OsgPointCloudRepresentation>
									   (node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

	EXPECT_EQ("SurgSim::Graphics::OsgPointCloudRepresentation", newOsgPointCloud->getClassName());
	EXPECT_NEAR(pointSize, newOsgPointCloud->getValue<double>("PointSize"), epsilon);
	EXPECT_TRUE(color.isApprox(newOsgPointCloud->getValue<Vector4d>("Color")));
}

}; // namespace Graphics
}; // namespace SurgSim

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

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/SurfaceMeshShape.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::EmptyData;
using SurgSim::DataStructures::TriangleMeshPlain;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::SurfaceMeshShape;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;

class SurfaceMeshShapeTest : public ::testing::Test
{
public:

	void SetUp()
	{
		m_radius = 2.34;
		m_thickness = 1e-2;
		m_center = Vector3d(1.3, 3.4, 5.6);
		m_expectedVolume = M_PI * m_radius * m_radius * m_thickness;
		m_expectedMatrix.setZero();
		m_expectedMatrix(0, 0) = m_expectedVolume * m_radius * m_radius / 4.0;
		m_expectedMatrix(1, 1) = m_expectedVolume * m_radius * m_radius / 4.0;
		m_expectedMatrix(2, 2) = m_expectedVolume * m_radius * m_radius / 2.0;
	}

	void TearDown()
	{
	}

	double m_radius;
	double m_thickness;
	Vector3d m_center;

	double m_expectedVolume;
	Matrix33d m_expectedMatrix;

	std::shared_ptr<TriangleMeshPlain> buildDiskZ(const Quaterniond& q, const Vector3d& center, double radius) const
	{
		size_t totalNumNodes = 501; // 1 center + lots on perimeter
		double deltaAngle = 2.0 * M_PI / static_cast<double>(totalNumNodes - 1);
		auto disk = std::make_shared<TriangleMeshPlain>();

		// Add the center point
		disk->addVertex(TriangleMeshPlain::VertexType(center));
		// Add the peripheral points
		for (size_t nodeId = 0; nodeId < totalNumNodes - 1; ++nodeId)
		{
			double angle = deltaAngle * nodeId;
			Vector3d p(m_radius * cos(angle), m_radius * sin(angle), 0.0);
			disk->addVertex(TriangleMeshPlain::VertexType(q._transformVector(p) + center));
		}

		// Define the triangles
		for (size_t triId = 1; triId < totalNumNodes - 1; ++triId)
		{
			std::array<size_t, 3> indices = {{ 0, triId, triId + 1}};
			disk->addTriangle(TriangleMeshPlain::TriangleType(indices));
		}
		return disk;
	}
};

TEST_F(SurfaceMeshShapeTest, EmptyMeshTest)
{
	TriangleMeshPlain emptyMesh;
	std::shared_ptr<SurfaceMeshShape> diskShape = std::make_shared<SurfaceMeshShape>(emptyMesh, m_thickness);

	EXPECT_NEAR(0.0, diskShape->getVolume(), 1e-9);
	EXPECT_TRUE(diskShape->getCenter().isZero());
	EXPECT_TRUE(diskShape->getSecondMomentOfVolume().isZero());
}

TEST_F(SurfaceMeshShapeTest, DiskShapeTest)
{
	std::shared_ptr<TriangleMeshPlain> diskMesh = buildDiskZ(Quaterniond::Identity(), m_center, m_radius);
	std::shared_ptr<SurfaceMeshShape> diskShape = std::make_shared<SurfaceMeshShape>(*diskMesh, m_thickness);

	EXPECT_NEAR(m_expectedVolume, diskShape->getVolume(), 1e-2);
	EXPECT_TRUE(diskShape->getCenter().isApprox(m_center, 1e-2));
	EXPECT_TRUE(diskShape->getSecondMomentOfVolume().isApprox(m_expectedMatrix, 1e-2));
}

TEST_F(SurfaceMeshShapeTest, NonAlignedDiskShapeTest)
{
	Quaterniond q(1.3, 5.3, -8.2, 2.4);
	q.normalize();
	std::shared_ptr<TriangleMeshPlain> diskMesh = buildDiskZ(q, m_center, m_radius);
	std::shared_ptr<SurfaceMeshShape> diskShape = std::make_shared<SurfaceMeshShape>(*diskMesh, m_thickness);

	Matrix33d rotatedExpectedMatrix = q.toRotationMatrix() * m_expectedMatrix * q.toRotationMatrix().transpose();

	EXPECT_NEAR(m_expectedVolume, diskShape->getVolume(), 1e-2);
	EXPECT_TRUE(diskShape->getCenter().isApprox(m_center, 1e-2));
	EXPECT_TRUE(diskShape->getSecondMomentOfVolume().isApprox(rotatedExpectedMatrix, 1e-2));
}


TEST_F(SurfaceMeshShapeTest, SerializationTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	const std::string fileName = "MeshShapeData/staple_collision.ply";
	auto surfaceMeshShape = std::make_shared<SurgSim::Math::SurfaceMeshShape>();
	surfaceMeshShape->load(fileName);

	// We chose to let YAML serialization only works with base class pointer.
	// i.e. We need to serialize 'surfaceMeshShape' via a SurgSim::Math::Shape pointer.
	// The usage YAML::Node node = surfaceMeshShape; will not compile.
	std::shared_ptr<SurgSim::Math::Shape> shape = surfaceMeshShape;

	YAML::Node node;
	ASSERT_NO_THROW(node = shape); // YAML::convert<std::shared_ptr<SurgSim::Math::Shape>> will be called.
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(1u, node.size());

	std::shared_ptr<SurgSim::Math::SurfaceMeshShape> newSurfaceMesh;
	ASSERT_NO_THROW(newSurfaceMesh = std::dynamic_pointer_cast<SurgSim::Math::SurfaceMeshShape>(
										 node.as<std::shared_ptr<SurgSim::Math::Shape>>()));

	EXPECT_EQ("SurgSim::Math::SurfaceMeshShape", newSurfaceMesh->getClassName());
	EXPECT_EQ(fileName, newSurfaceMesh->getFileName());
	EXPECT_EQ(surfaceMeshShape->getNumVertices(), newSurfaceMesh->getNumVertices());
	EXPECT_EQ(surfaceMeshShape->getNumEdges(), newSurfaceMesh->getNumEdges());
	EXPECT_EQ(surfaceMeshShape->getNumTriangles(), newSurfaceMesh->getNumTriangles());
}

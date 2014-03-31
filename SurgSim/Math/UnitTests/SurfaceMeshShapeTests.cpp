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
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Shapes.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::EmptyData;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::SurfaceMeshShape;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;

class SurfaceMeshShapeTest : public ::testing::Test
{
public:
	typedef SurgSim::DataStructures::TriangleMeshBase<EmptyData, EmptyData, EmptyData> TriangleMeshBase;
	typedef SurgSim::DataStructures::MeshElement<2, EmptyData> EdgeElement;
	typedef SurgSim::DataStructures::MeshElement<3, EmptyData> TriangleElement;

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

	TriangleMeshBase buildDiskZ(const Quaterniond& q, const Vector3d& center, double radius) const
	{
		size_t totalNumNodes = 501; // 1 center + lots on perimeter
		double deltaAngle = 2.0 * M_PI / static_cast<double>(totalNumNodes - 1);
		TriangleMeshBase disk;

		// Add the center point
		disk.addVertex(TriangleMeshBase::VertexType(center));
		// Add the peripheral points
		for (size_t nodeId = 0; nodeId < totalNumNodes - 1; ++nodeId)
		{
			double angle = deltaAngle * nodeId;
			Vector3d p(m_radius * cos(angle), m_radius * sin(angle), 0.0);
			disk.addVertex(TriangleMeshBase::VertexType(q._transformVector(p) + center));
		}

		// Define the triangles
		for (size_t triId = 1; triId < totalNumNodes - 1; ++triId)
		{
			std::array<unsigned int, 3> indices = {{ 0, triId, triId + 1}};
			disk.addTriangle(TriangleMeshBase::TriangleType(indices));
		}
		return disk;
	}
};

TEST_F(SurfaceMeshShapeTest, EmptyMeshTest)
{
	TriangleMeshBase emptyMesh;
	std::shared_ptr<SurfaceMeshShape> diskShape = std::make_shared<SurfaceMeshShape>(emptyMesh, m_thickness);

	EXPECT_NEAR(0.0, diskShape->getVolume(), 1e-9);
	EXPECT_TRUE(diskShape->getCenter().isZero());
	EXPECT_TRUE(diskShape->getSecondMomentOfVolume().isZero());
}

TEST_F(SurfaceMeshShapeTest, DiskShapeTest)
{
	TriangleMeshBase diskMesh = buildDiskZ(Quaterniond::Identity(), m_center, m_radius);
	std::shared_ptr<SurfaceMeshShape> diskShape = std::make_shared<SurfaceMeshShape>(diskMesh, m_thickness);

	EXPECT_NEAR(m_expectedVolume, diskShape->getVolume(), 1e-2);
	EXPECT_TRUE(diskShape->getCenter().isApprox(m_center, 1e-2));
	EXPECT_TRUE(diskShape->getSecondMomentOfVolume().isApprox(m_expectedMatrix, 1e-2));
}

TEST_F(SurfaceMeshShapeTest, NonAlignedDiskShapeTest)
{
	Quaterniond q(1.3, 5.3, -8.2, 2.4);
	q.normalize();
	TriangleMeshBase diskMesh = buildDiskZ(q, m_center, m_radius);
	std::shared_ptr<SurfaceMeshShape> diskShape = std::make_shared<SurfaceMeshShape>(diskMesh, m_thickness);

	Matrix33d rotatedExpectedMatrix = q.toRotationMatrix() * m_expectedMatrix * q.toRotationMatrix().transpose();

	EXPECT_NEAR(m_expectedVolume, diskShape->getVolume(), 1e-2);
	EXPECT_TRUE(diskShape->getCenter().isApprox(m_center, 1e-2));
	EXPECT_TRUE(diskShape->getSecondMomentOfVolume().isApprox(rotatedExpectedMatrix, 1e-2));
}

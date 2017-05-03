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

#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Aabb.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/SegmentMeshShape.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::EmptyData;
using SurgSim::DataStructures::SegmentMeshPlain;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::SegmentMeshShape;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;

class SegmentMeshShapeTest : public ::testing::Test
{
public:
	std::shared_ptr<SegmentMeshPlain> build(const Vector3d& start, const Vector3d& direction,
											size_t numVertices = 10) const
	{
		auto mesh = std::make_shared<SegmentMeshPlain>();

		// Add the vertices
		for (size_t i = 0; i < numVertices; ++i)
		{
			mesh->addVertex(SegmentMeshPlain::VertexType(start + direction * static_cast<double>(i)));
		}

		// Define the edges
		for (size_t i = 0; i < numVertices - 1; ++i)
		{
			std::array<size_t, 2> indices = {{i, i + 1}};
			mesh->addEdge(SegmentMeshPlain::EdgeType(indices));
		}
		return mesh;
	}
};

TEST_F(SegmentMeshShapeTest, EmptyMeshTest)
{
	SegmentMeshPlain emptyMesh;
	EXPECT_NO_THROW(SurgSim::Math::SegmentMeshShape(emptyMesh, 1.0));
}

TEST_F(SegmentMeshShapeTest, MeshTypeTest)
{
	SegmentMeshPlain emptyMesh;
	SurgSim::Math::SegmentMeshShape segmentShape(emptyMesh, 1.0);
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_SEGMENTMESH, segmentShape.getType());
}

TEST_F(SegmentMeshShapeTest, ValidTest)
{
	SegmentMeshPlain emptyMesh;
	SurgSim::Math::SegmentMeshShape shape(emptyMesh, 1.0);
	EXPECT_TRUE(shape.isValid());
	shape.setRadius(SurgSim::Math::Geometry::DistanceEpsilon / 2.0);
	EXPECT_FALSE(shape.isValid());

	EXPECT_TRUE(shape.isTransformable());
}

TEST_F(SegmentMeshShapeTest, AabbTest)
{
	std::shared_ptr<SegmentMeshPlain> mesh = build(Vector3d(-10.0, -10.0, -10.0), Vector3d(10.0, 10.0, 10.0), 3);
	std::shared_ptr<SegmentMeshShape> shape = std::make_shared<SegmentMeshShape>(*mesh, 3.0);

	SurgSim::Math::Aabbd expected(Vector3d(-13.0, -13.0, -13.0), Vector3d(13.0, 13.0, 13.0));

	shape->update();
	EXPECT_TRUE(expected.isApprox(shape->getAabbTree()->getAabb()));
}

TEST_F(SegmentMeshShapeTest, TransformTest)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRotationQuaternion;

	std::shared_ptr<SegmentMeshPlain> mesh = build(Vector3d(-10.0, -10.0, -10.0), Vector3d(10.0, 10.0, 10.0), 3);
	std::shared_ptr<SegmentMeshShape> shape = std::make_shared<SegmentMeshShape>(*mesh, 3.0);

	// Transform into a new mesh
	auto newShape = std::dynamic_pointer_cast<SegmentMeshShape>(shape->getTransformed(
						makeRigidTransform(makeRotationQuaternion(M_PI_2,
										   Vector3d(0.0, 1.0, 0.0)), Vector3d(0.0, 10.0, 0.0))));

	EXPECT_TRUE(Vector3d(-10.0, 0.0, 10.0).isApprox(newShape->getVertex(0).position));
	EXPECT_TRUE(Vector3d(0.0, 10.0, 0.0).isApprox(newShape->getVertex(1).position));
	EXPECT_TRUE(Vector3d(10.0, 20.0, -10.0).isApprox(newShape->getVertex(2).position));

	// Transform existing mesh
	mesh->transform(
		makeRigidTransform(makeRotationQuaternion(M_PI_2, Vector3d(0.0, 1.0, 0.0)), Vector3d(0.0, 10.0, 0.0)));

	// Verify
	EXPECT_TRUE(Vector3d(-10.0, 0.0, 10.0).isApprox(mesh->getVertex(0).position));
	EXPECT_TRUE(Vector3d(0.0, 10.0, 0.0).isApprox(mesh->getVertex(1).position));
	EXPECT_TRUE(Vector3d(10.0, 20.0, -10.0).isApprox(mesh->getVertex(2).position));
}

TEST_F(SegmentMeshShapeTest, LoadShape)
{
	auto mesh = std::make_shared<SegmentMeshShape>();
	SurgSim::Framework::ApplicationData data("config.txt");

	mesh->load("segmentmesh.ply", data);

	ASSERT_EQ(4u, mesh->getNumVertices());
	ASSERT_EQ(3u, mesh->getNumEdges());

	auto edge = mesh->getEdge(0);
	ASSERT_EQ(0u, edge.verticesId[0]);
	ASSERT_EQ(1u, edge.verticesId[1]);

	edge = mesh->getEdge(1);
	ASSERT_EQ(1u, edge.verticesId[0]);
	ASSERT_EQ(2u, edge.verticesId[1]);

	edge = mesh->getEdge(2);
	ASSERT_EQ(2u, edge.verticesId[0]);
	ASSERT_EQ(3u, edge.verticesId[1]);

	EXPECT_DOUBLE_EQ(1.234, mesh->getRadius());
}

TEST_F(SegmentMeshShapeTest, GetPose)
{
	auto segmentMeshPlain = std::make_shared<SurgSim::DataStructures::SegmentMeshPlain>();
	const int numPoints = 10;
	for (int i = 0; i < numPoints; i++)
	{
		segmentMeshPlain->addVertex(SurgSim::DataStructures::SegmentMeshPlain::VertexType(Vector3d::Random()));
	}
	auto segmentMeshShape = std::make_shared<SegmentMeshShape>(*segmentMeshPlain);

	const auto& initial = segmentMeshShape->getInitialVertices();
	for (size_t i = 0; i < segmentMeshShape->getNumVertices(); ++i)
	{
		EXPECT_TRUE(segmentMeshShape->getVertex(i).position.isApprox(initial.getVertex(i).position));
	}

	const auto pose = SurgSim::Math::makeRigidTransform(
		SurgSim::Math::makeRotationQuaternion(0.1, Vector3d(0.1, 0.2, -0.1)),
		Vector3d(3.0, 4.0, -5.0));
	segmentMeshShape->setPose(pose);
	for (size_t i = 0; i < segmentMeshShape->getNumVertices(); ++i)
	{
		EXPECT_TRUE(segmentMeshShape->getVertex(i).position.isApprox(pose * initial.getVertex(i).position));
	}

	segmentMeshShape->setPose(SurgSim::Math::RigidTransform3d::Identity());
	for (size_t i = 0; i < segmentMeshShape->getNumVertices(); ++i)
	{
		EXPECT_TRUE(segmentMeshShape->getVertex(i).position.isApprox(initial.getVertex(i).position));
	}

	segmentMeshShape->setPose(pose);
	segmentMeshShape->setInitialVertices(*segmentMeshShape);
	const auto& newInitial = segmentMeshShape->getInitialVertices();
	for (size_t i = 0; i < segmentMeshShape->getNumVertices(); ++i)
	{
		EXPECT_TRUE(segmentMeshShape->getVertex(i).position.isApprox(newInitial.getVertex(i).position));
	}
}

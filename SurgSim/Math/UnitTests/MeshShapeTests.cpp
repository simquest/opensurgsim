//// This file is a part of the OpenSurgSim project.
//// Copyright 2013, SimQuest Solutions Inc.
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////     http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//// See the License for the specific language governing permissions and
//// limitations under the License.

#include <time.h>

#include <gtest/gtest.h>

#include <SurgSim/Math/Vector.h>

#include <SurgSim/Math/Shapes.h>

// CUBE
//     3*----------*2
//     /          /|
//   7*----------* |
//    |         6| |
//    | *0       | *1
//    |          |/
//   4*----------*5

static const int cubeNumPoints = 8;
static const SurgSim::Math::Vector3d cubePoints[8]=
{
	SurgSim::Math::Vector3d(-1.0/2.0, -1.0/2.0, -1.0/2.0),
	SurgSim::Math::Vector3d( 1.0/2.0, -1.0/2.0, -1.0/2.0),
	SurgSim::Math::Vector3d( 1.0/2.0,  1.0/2.0, -1.0/2.0),
	SurgSim::Math::Vector3d(-1.0/2.0,  1.0/2.0, -1.0/2.0),

	SurgSim::Math::Vector3d(-1.0/2.0, -1.0/2.0,  1.0/2.0),
	SurgSim::Math::Vector3d( 1.0/2.0, -1.0/2.0,  1.0/2.0),
	SurgSim::Math::Vector3d( 1.0/2.0,  1.0/2.0,  1.0/2.0),
	SurgSim::Math::Vector3d(-1.0/2.0,  1.0/2.0,  1.0/2.0)
};

static const int cubeNumEdges = 12;
static const int cubeEdges[12][2] =
{
	{0, 1}, {3, 2}, {4, 5}, {7, 6}, // +X
	{0, 3}, {1, 2}, {4, 7}, {5 ,6}, // +Y
	{0, 4}, {1, 5}, {2, 6}, {3, 7}  // +Z
};

static const int cubeNumTriangles = 12;
static const int cubeTrianglesCCW[12][3] =
{
	{6, 2, 3}, {6, 3, 7}, // Top    ( 0  1  0) [6237]
	{0, 1, 5}, {0, 5, 4}, // Bottom ( 0 -1  0) [0154]
	{4, 5, 6}, {4, 6, 7}, // Front  ( 0  0  1) [4567]
	{0, 3, 2}, {0, 2, 1}, // Back   ( 0  0 -1) [0321]
	{1, 2, 6}, {1, 6, 5}, // Right  ( 1  0  0) [1265]
	{0, 4, 7}, {0, 7, 3}  // Left   (-1  0  0) [0473]
};

class EmptyData
{
public:
	bool operator ==(const EmptyData& e) const
	{
		return true;
	}
};

class CubeMeshTest : public ::testing::Test
{
public:
	typedef SurgSim::DataStructures::TriangleMesh<EmptyData,EmptyData,EmptyData> TriangleMesh;
	typedef SurgSim::DataStructures::MeshElement<2,EmptyData> EdgeElement;
	typedef SurgSim::DataStructures::MeshElement<3,EmptyData> TriangleElement;

	void SetUp()
	{
		m_numIterations = 100;

		srand((unsigned int)time(nullptr));
	}

	void TearDown()
	{
	}

	/// Number of iterations to test
	int m_numIterations;
};

TEST_F(CubeMeshTest, MeshCubeVSBoxTest)
{
	for (int iterationID = 0; iterationID < m_numIterations; iterationID++)
	{
		double lx = 10.0 * static_cast<double>(rand())/static_cast<double>(RAND_MAX) + 1.0;
		double ly = 10.0 * static_cast<double>(rand())/static_cast<double>(RAND_MAX) + 1.0;
		double lz = 10.0 * static_cast<double>(rand())/static_cast<double>(RAND_MAX) + 1.0;

		// Cube
		std::shared_ptr<TriangleMesh> mesh = std::make_shared<TriangleMesh>();
		for (int i = 0; i < cubeNumPoints; i++)
		{
			EmptyData emptyData;
			SurgSim::Math::Vector3d p;
			p[0] = cubePoints[i][0] * lx;
			p[1] = cubePoints[i][1] * ly;
			p[2] = cubePoints[i][2] * lz;
			TriangleMesh::VertexType v(p, emptyData);
			mesh->addVertex(v);
		}
		for (int i = 0; i < cubeNumEdges; i++)
		{
			EmptyData emptyData;
			std::array<unsigned int,2> edgePoints;
			for (int j = 0; j < 2; j++)
			{
				edgePoints[j] = cubeEdges[i][j];
			}
			EdgeElement edgeElement(edgePoints, emptyData);
			TriangleMesh::EdgeType e(edgeElement);
			mesh->addEdge(e);
		}
		for (int i = 0; i < cubeNumTriangles; i++)
		{
			EmptyData emptyData;
			std::array<unsigned int,3> trianglePoints;
			for (int j = 0; j < 3; j++)
			{
				trianglePoints[j] = cubeTrianglesCCW[i][j];
			}
			TriangleElement triangleElement(trianglePoints, emptyData);
			TriangleMesh::TriangleType t(triangleElement);
			mesh->addTriangle(t);
		}

		SurgSim::Math::MeshShape<EmptyData, EmptyData, EmptyData> boxMesh(mesh);

		SurgSim::Math::BoxShape boxShape(lx, ly, lz);

		EXPECT_NEAR(boxShape.getVolume(), boxMesh.getVolume(), 1e-8);
		EXPECT_TRUE((boxShape.getCenter() - boxMesh.getCenter()).isZero());
		EXPECT_TRUE(boxShape.getSecondMomentOfVolume().isApprox(boxMesh.getSecondMomentOfVolume(), 1e-8));
	}
}

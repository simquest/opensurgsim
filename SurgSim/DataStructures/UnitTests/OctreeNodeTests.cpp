// This file is a part of the OpenSurgSim project.
// Copyright 2012-2016, SimQuest Solutions Inc.
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
/// Tests for the OctreeNode class.

#include "gtest/gtest.h"
#include <array>
#include <memory>
#include <string>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/OctreeNode.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::EmptyData;
using SurgSim::DataStructures::OctreeNode;
using SurgSim::Math::Vector3d;

struct MockData
{
	int mockInt = 0;
	double mockDouble = std::numeric_limits<double>::signaling_NaN();
	std::string mockString;
};

typedef OctreeNode<MockData> OctreeNodeType;

template<>
std::string OctreeNodeType::m_className = "OctreeNode<MockData>";

namespace
{
const double epsilon = 1e-14;
}

namespace SurgSim
{
namespace DataStructures
{

TEST(OctreeNodeTests, CanConstruct)
{
	SurgSim::Math::Aabbd boundingBox(Vector3d::Zero(), Vector3d::Ones());

	EXPECT_NO_THROW({OctreeNodeType octree(boundingBox);});
	EXPECT_NO_THROW(std::make_shared<OctreeNodeType>(boundingBox));
}

TEST(OctreeNodeTests, InitialValues)
{
	SurgSim::Math::Aabbd expectedBoundingBox(Vector3d::Zero(), Vector3d::Ones());
	OctreeNodeType octree(expectedBoundingBox);

	EXPECT_FALSE(octree.isActive());
	EXPECT_FALSE(octree.hasChildren());
	EXPECT_TRUE(expectedBoundingBox.isApprox(octree.getBoundingBox()));

	auto children = octree.getChildren();
	for (auto child = children.cbegin(); child != children.cend(); ++child)
	{
		EXPECT_EQ(nullptr, *child);
	}
}

TEST(OctreeNodeTests, SetIsActive)
{
	SurgSim::Math::Aabbd expectedBoundingBox(Vector3d::Zero(), Vector3d::Ones());
	OctreeNodeType octree(expectedBoundingBox);

	octree.setIsActive(true);
	EXPECT_TRUE(octree.isActive());

	octree.setIsActive(false);
	EXPECT_FALSE(octree.isActive());
}

TEST(OctreeNodeTests, Subdivide)
{
	SurgSim::Math::Aabbd boundingBox(Vector3d::Zero(), Vector3d::Ones() * 16.0);
	OctreeNodeType octree(boundingBox);

	EXPECT_FALSE(octree.hasChildren());
	EXPECT_FALSE(octree.isActive());
	octree.subdivide();
	EXPECT_TRUE(octree.hasChildren());
	EXPECT_FALSE(octree.isActive());

	auto children = octree.getChildren();
	for (auto child = children.cbegin(); child != children.cend(); ++child)
	{
		ASSERT_NE(nullptr, *child);
		EXPECT_FALSE((*child)->isActive());
		EXPECT_FALSE((*child)->hasChildren());
	}

	std::array<SurgSim::Math::Aabbd, 8> expectedBoxes = {{
			SurgSim::Math::Aabbd(Vector3d(0.0, 0.0, 0.0), Vector3d(8.0, 8.0, 8.0)),
			SurgSim::Math::Aabbd(Vector3d(0.0, 0.0, 8.0), Vector3d(8.0, 8.0, 16.0)),
			SurgSim::Math::Aabbd(Vector3d(0.0, 8.0, 0.0), Vector3d(8.0, 16.0, 8.0)),
			SurgSim::Math::Aabbd(Vector3d(0.0, 8.0, 8.0), Vector3d(8.0, 16.0, 16.0)),
			SurgSim::Math::Aabbd(Vector3d(8.0, 0.0, 0.0), Vector3d(16.0, 8.0, 8.0)),
			SurgSim::Math::Aabbd(Vector3d(8.0, 0.0, 8.0), Vector3d(16.0, 8.0, 16.0)),
			SurgSim::Math::Aabbd(Vector3d(8.0, 8.0, 0.0), Vector3d(16.0, 16.0, 8.0)),
			SurgSim::Math::Aabbd(Vector3d(8.0, 8.0, 8.0), Vector3d(16.0, 16.0, 16.0))
		}
	};
	for (auto expectedBox = expectedBoxes.cbegin(); expectedBox != expectedBoxes.cend(); ++expectedBox)
	{
		bool boxFound = false;
		for (auto child = children.cbegin(); child != children.cend(); ++child)
		{
			if (expectedBox->isApprox((*child)->getBoundingBox()))
			{
				boxFound = true;
				break;
			}
		}
		EXPECT_TRUE(boxFound);
	}
}

int countOctreeLevels(std::shared_ptr<OctreeNodeType> node)
{
	if (node->hasChildren())
	{
		auto children = node->getChildren();
		int maxLevels = 0;
		for (auto child = children.cbegin(); child != children.cend(); ++child)
		{
			if ((*child)->isActive())
			{
				int levels = countOctreeLevels(*child);
				if (levels > maxLevels)
				{
					maxLevels = levels;
				}
			}
		}
		return maxLevels + 1;
	}
	return 1;
}

TEST(OctreeNodeTests, AddNodes)
{
	SurgSim::Math::Aabbd boundingBox(Vector3d::Ones() * (-8.0), Vector3d::Ones() * 8.0);
	std::shared_ptr<OctreeNodeType> octree = std::make_shared<OctreeNodeType>(boundingBox);

	const int levels = 5;
	MockData data = {1, 3.14, "string"};

	EXPECT_FALSE(octree->hasChildren());
	EXPECT_FALSE(octree->isActive());

	EXPECT_TRUE(octree->addData(Vector3d(1.0, 1.0, 1.0), levels, data));
	EXPECT_TRUE(octree->addData(Vector3d(-4.0, 5.0, -7.0), levels, data));

	EXPECT_TRUE(octree->hasChildren());
	EXPECT_TRUE(octree->isActive());
	EXPECT_EQ(5, countOctreeLevels(octree));

	int numActive = 0;
	auto children = octree->getChildren();
	for (auto child = children.cbegin(); child != children.cend(); ++child)
	{
		if ((*child)->isActive())
		{
			numActive++;
		}
	}
	EXPECT_EQ(2, numActive);
}

TEST(OctreeNodeTests, Data)
{
	SurgSim::Math::Aabbd boundingBox(Vector3d::Ones() * (-8.0), Vector3d::Ones() * 8.0);
	OctreeNodeType octree(boundingBox);

	const int levels = 1;
	MockData expectedData = {1, 3.14, "string"};

	EXPECT_FALSE(octree.hasChildren());
	EXPECT_FALSE(octree.isActive());
	EXPECT_TRUE(octree.addData(Vector3d(1.0, 1.0, 1.0), levels, expectedData));
	EXPECT_FALSE(octree.hasChildren());
	EXPECT_TRUE(octree.isActive());

	EXPECT_EQ(expectedData.mockInt, octree.data.mockInt);
	EXPECT_NEAR(expectedData.mockDouble, octree.data.mockDouble, epsilon);
	EXPECT_EQ(expectedData.mockString, octree.data.mockString);
}

TEST(OctreeNodeTests, OctreePath)
{
	SurgSim::Math::Aabbd boundingBox(Vector3d::Ones() * (-8.0), Vector3d::Ones() * 8.0);
	std::shared_ptr<OctreeNodeType> octree = std::make_shared<OctreeNodeType>(boundingBox);

	SurgSim::DataStructures::OctreePath path;
	EXPECT_NO_THROW(octree->getNode(path));
	EXPECT_EQ(octree, octree->getNode(path));

	octree->subdivide();
	path.push_back(3);
	EXPECT_NO_THROW(octree->getNode(path));
	EXPECT_NE(nullptr, octree->getNode(path));

	auto previous = octree->getNode(path);

	path.push_back(1);
	EXPECT_THROW(octree->getNode(path), SurgSim::Framework::AssertionFailure);

	// Should return the last valid node on the path
	EXPECT_NO_THROW(octree->getNode(path, true));
	EXPECT_EQ(previous, octree->getNode(path, true));
}

struct Data1
{
	std::string name;
};

template <>
std::string OctreeNode<Data1>::m_className = "OctreeNode<Data1>";

struct Data2
{
	double value;
};

template <>
std::string OctreeNode<Data2>::m_className = "OctreeNode<Data2>";

TEST(OctreeNodeTests, CopyConstructor)
{


	SurgSim::Math::Aabbd boundingBox(Vector3d::Zero(), 2 * Vector3d::Ones());
	std::shared_ptr<OctreeNode<Data1>> octree1 = std::make_shared<OctreeNode<Data1>>(boundingBox);
	Data1 dataRoot = {"root"};
	octree1->addData(Vector3d(1.0, 1.0, 1.0), 1, dataRoot);
	Data1 dataChild = {"child"};
	octree1->addData(Vector3d(0.5, 0.5, 0.5), 2, dataChild);

	{
		SCOPED_TRACE("Copying with different Data Type");
		std::shared_ptr<OctreeNode<Data2>> octree2 = std::make_shared<OctreeNode<Data2>>(*octree1);
		ASSERT_NE(nullptr, octree2);
		EXPECT_TRUE(octree1->getBoundingBox().isApprox(octree2->getBoundingBox()));
		EXPECT_EQ(octree1->hasChildren(), octree2->hasChildren());
		EXPECT_EQ(octree1->isActive(), octree2->isActive());
		for (size_t i = 0; i < 8; i++)
		{
			if (octree1->getChild(i) == nullptr)
			{
				EXPECT_EQ(nullptr, octree2->getChild(i));
			}
			else
			{
				ASSERT_NE(nullptr, octree2->getChild(i));
				EXPECT_TRUE(octree1->getChild(i)->getBoundingBox().isApprox(octree2->getChild(i)->getBoundingBox()));
				EXPECT_EQ(octree1->getChild(i)->hasChildren(), octree2->getChild(i)->hasChildren());
				EXPECT_EQ(octree1->getChild(i)->isActive(), octree2->getChild(i)->isActive());
			}
		}
	}

	{
		SCOPED_TRACE("Copying with same Data Type");
		std::shared_ptr<OctreeNode<Data1>> octree2 = std::make_shared<OctreeNode<Data1>>(*octree1);
		ASSERT_NE(nullptr, octree2);
		EXPECT_TRUE(octree1->getBoundingBox().isApprox(octree2->getBoundingBox()));
		EXPECT_EQ(octree1->hasChildren(), octree2->hasChildren());
		EXPECT_EQ(octree1->isActive(), octree2->isActive());
		EXPECT_EQ(octree1->data.name, octree2->data.name);
		for (size_t i = 0; i < 8; i++)
		{
			if (octree1->getChild(i) == nullptr)
			{
				EXPECT_EQ(nullptr, octree2->getChild(i));
			}
			else
			{
				ASSERT_NE(nullptr, octree2->getChild(i));
				EXPECT_TRUE(octree1->getChild(i)->getBoundingBox().isApprox(octree2->getChild(i)->getBoundingBox()));
				EXPECT_EQ(octree1->getChild(i)->hasChildren(), octree2->getChild(i)->hasChildren());
				EXPECT_EQ(octree1->getChild(i)->isActive(), octree2->getChild(i)->isActive());
				EXPECT_EQ(octree1->getChild(i)->data.name, octree2->getChild(i)->data.name);
			}
		}
	}
}

TEST(OctreeNodeTests, EmptyData)
{
	SurgSim::Math::Aabbd boundingBox(Vector3d::Zero(), Vector3d::Ones());

	EXPECT_NO_THROW({OctreeNode<EmptyData> octree(boundingBox);});
	EXPECT_NO_THROW(std::make_shared<OctreeNode<EmptyData>>(boundingBox));
}

TEST(OctreeNodeTests, DoLoadOctree)
{
	SurgSim::Framework::ApplicationData appData("config.txt");
	auto octree = std::make_shared<OctreeNode<SurgSim::DataStructures::EmptyData>>();
	ASSERT_NO_THROW(octree->load("Geometry/staple.ply", appData));

	ASSERT_TRUE(nullptr != octree);
	auto boundingBox = octree->getBoundingBox();

	SurgSim::Math::Vector3d boundingMin(-0.00207699998282, -0.00532899983227, -0.000403999991249);
	SurgSim::Math::Vector3d boundingMax(0.01392300001718, 0.01067100016773, 0.015596000008751);
	EXPECT_TRUE(boundingMin.isApprox(boundingBox.min()));
	EXPECT_TRUE(boundingMax.isApprox(boundingBox.max()));

	EXPECT_TRUE(octree->isActive());
	EXPECT_TRUE(octree->hasChildren());

	EXPECT_TRUE(octree->getChild(0)->isActive());
	EXPECT_TRUE(octree->getChild(0)->hasChildren());

	EXPECT_TRUE(octree->getChild(0)->getChild(2)->isActive());
	EXPECT_TRUE(octree->getChild(0)->getChild(2)->hasChildren());

	EXPECT_TRUE(octree->getChild(0)->getChild(2)->getChild(2)->isActive());

	Vector3d leafSize = octree->getChild(0)->getChild(0)->getChild(0)->getChild(0)->getBoundingBox().sizes();
	EXPECT_TRUE(leafSize.isApprox(Vector3d(0.001, 0.001, 0.001)));
}

TEST(OctreeNodeTests, NeighborhoodTestSimple)
{
	OctreePath path;
	{
		SCOPED_TRACE("No direction should not fail");
		std::array<Symbol, 3> direction = { SYMBOL_HALT, SYMBOL_HALT, SYMBOL_HALT};
		EXPECT_NO_THROW(getNeighbor(path, direction));
	}

	path.push_back(0);
	{
		SCOPED_TRACE("Right of 0 should be 1");
		std::array<Symbol, 3> direction = { SYMBOL_RIGHT, SYMBOL_HALT, SYMBOL_HALT};
		EXPECT_NO_THROW(getNeighbor(path, direction));
		auto result = getNeighbor(path, direction);
		ASSERT_EQ(1u, result.size());
		EXPECT_EQ(1u, result[0]);
	}

	{
		SCOPED_TRACE("Left of 0 with not levels is nothing");
		std::array<Symbol, 3> direction = { SYMBOL_LEFT, SYMBOL_HALT, SYMBOL_HALT};
		EXPECT_NO_THROW(getNeighbor(path, direction));
		auto result = getNeighbor(path, direction);
		ASSERT_EQ(0u, result.size());
	}
}


// This verifies the basic non-boundary crossing neighborhoods
TEST(OctreeNodeTests, NeigborhoodPlainFaces)
{
	size_t testValues[24][3] =
	{
		0, SYMBOL_RIGHT, 1,
		0, SYMBOL_UP, 2,
		0, SYMBOL_FRONT, 4,

		1, SYMBOL_LEFT, 0,
		1, SYMBOL_UP, 3,
		1, SYMBOL_FRONT, 5,

		2, SYMBOL_RIGHT, 3,
		2, SYMBOL_DOWN, 0,
		2, SYMBOL_FRONT, 6,

		3, SYMBOL_LEFT, 2,
		3, SYMBOL_DOWN, 1,
		3, SYMBOL_FRONT, 7,

		4, SYMBOL_RIGHT, 5,
		4, SYMBOL_UP, 6,
		4, SYMBOL_BACK, 0,

		5, SYMBOL_LEFT, 4,
		5, SYMBOL_UP, 7,
		5, SYMBOL_BACK, 1,

		6, SYMBOL_RIGHT, 7,
		6, SYMBOL_DOWN, 4,
		6, SYMBOL_BACK, 2,

		7, SYMBOL_LEFT, 6,
		7, SYMBOL_DOWN, 5,
		7, SYMBOL_BACK, 3
	};

	for (size_t i = 0; i < 24; ++i)
	{
		OctreePath path(1);
		path[0] = testValues[i][0];
		std::array<Symbol, 3> direction = { static_cast<Symbol>(testValues[i][1]), SYMBOL_HALT, SYMBOL_HALT};

		auto result = getNeighbor(path, direction);
		ASSERT_EQ(1u, result.size()) << "For row " << i;
		EXPECT_EQ(testValues[i][2], result[0]) << "For row " << i;
	}
}

/// Verifies face neighbors for one set of second level nodes, this should exercise all of the state table,
/// the expected data was calculated manually
TEST(OctreeNodeTests, AllFaceNeighbors)
{
	size_t expected[8][7][2] =
	{
		// Node	Down	Up		Right	Left	Back	Front
		7, 0,	5, 2,	7, 2,	7, 1,	6, 1,	3, 4,	7, 4,
		6, 1,	4, 3,	6, 3,	7, 0,	6, 0,	2, 5,	6, 5,
		5, 2,	5, 0,	7, 0,	5, 3,	4, 3,	1, 6,	5, 6,
		4, 3, 	6, 1,	4, 1,	5, 2,	4, 2,	0, 7,	4, 7,
		3, 4,	1, 6,	3, 6,	2, 5,	3, 5,	3, 0,	7, 0,
		2, 5,	0, 7,	2, 7,	3, 4,	2, 4,	2, 1,	6, 1,
		1, 6,	1, 4, 	3, 4, 	1, 7, 	0, 7,	1, 2, 	5, 2,
		0, 7,	0, 5,	2, 5,	1, 6,	0, 6,   0, 3,	4, 3
	};


	size_t checkSum = 0;
	for (int i = 0; i < 8; ++i)
	{
		OctreePath node;
		node.push_back(expected[i][0][0]);
		node.push_back(expected[i][0][1]);

		auto neighbors = getNeighbors(node, NEIGHBORHOOD_FACE);
		checkSum += neighbors.size();

		for (int j = 0; j < 6; ++j)
		{
			OctreePath neighbor;
			neighbor.push_back(expected[i][1 + j][0]);
			neighbor.push_back(expected[i][1 + j][1]);

			EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), neighbor) != neighbors.end())
					<< "Node # " << i << "Neighbor #" << j << " not found";

		}
	}

	EXPECT_EQ(48u, checkSum);
}


// This verifies edge neighbours for one node
TEST(OctreeNodeTests, EdgeNeighbors)
{
	OctreePath path;
	path.push_back(1);
	path.push_back(6);

	auto neighbors = getNeighbors(path, NEIGHBORHOOD_EDGE);
	const size_t expectedCount = 12;
	EXPECT_EQ(expectedCount, neighbors.size());

	// Expected coordinates of face neighbors for the node 1/6
	size_t expected[expectedCount][2] =
	{
		// Upper Ring
		3, 5, // Up Right
		2, 5, // Up Left
		3, 0, // Up Back
		7, 0, // Up Front

		// Lower Ring
		1, 5, // Down Right
		0, 5, // Down Left
		1, 0, // Down Back
		5, 0, // Down Front

		// Horizontals
		1, 3, // Back Right
		0, 3, // Back Left
		5, 3, // Front Right
		4, 3, // Front Left
	};

	for (size_t i = 0; i < expectedCount; ++i)
	{
		OctreePath path;
		path.push_back(expected[i][0]);
		path.push_back(expected[i][1]);

		EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), path) != neighbors.end())
				<< "Item #" << i << " not found";
	}

}

// This verifies corner neighbours for one node
TEST(OctreeNodeTests, VertexNeighbors)
{
	OctreePath path;
	path.push_back(1);
	path.push_back(6);

	auto neighbors = getNeighbors(path, NEIGHBORHOOD_VERTEX);
	const size_t expectedCount = 8;

	EXPECT_EQ(expectedCount, neighbors.size());

	// Expected coordinates of face neighbors for the node 1/6
	size_t expected[expectedCount][2] =
	{
		// Lower Ring
		5, 1, // Down Right Front
		1, 1, // Down Right Back
		4, 1, // Down Left Front
		0, 1, // Down Left Back

		// Upper Ring
		7, 1, // Up Right Front
		3, 1, // Up Right Back
		6, 1, // Up Left Front
		2, 1, // Up Left Back
	};

	for (size_t i = 0; i < expectedCount; ++i)
	{
		OctreePath path;
		path.push_back(expected[i][0]);
		path.push_back(expected[i][1]);

		EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), path) != neighbors.end())
				<< "Item #" << i << " not found";
	}
}


TEST(OctreeNodeTests, AllNeighbors)
{
	// All of these nodes are interior, they should return 26 neighbors
	size_t nodes[8][2] =
	{
		7, 0, 6, 1, 5, 2, 4, 3, 3, 4, 2, 5, 1, 6, 0, 7
	};

	for (int i = 0; i < 8; ++i)
	{
		OctreePath path;
		path.push_back(nodes[i][0]);
		path.push_back(nodes[i][1]);

		auto neighbors = getNeighbors(path, NEIGHBORHOOD_FACE | NEIGHBORHOOD_EDGE | NEIGHBORHOOD_VERTEX);

		EXPECT_EQ(26u, neighbors.size());
	}
}

TEST(OctreeNodeTests, IncompleteNeighbors)
{
	static const size_t numNodes = 16;
	size_t nodes[numNodes][3] =
	{
		//Node, Expected Vertex Neighbors
		0, 0, 1, // These are the cube corner pieces
		1, 1, 1,
		2, 2, 1,
		3, 3, 1,
		4, 4, 1,
		5, 5, 1,
		6, 6, 1,
		7, 7, 1,
		5, 4, 2, // These are on the outside cube edge
		3, 2, 2,
		1, 5, 2,
		0, 4, 2,
		3, 6, 4, // These are on an outside surface
		1, 4, 4,
		4, 2, 4,
		7, 2, 4
	};

	for (size_t i = 0; i < numNodes; ++i)
	{
		OctreePath path;
		path.push_back(nodes[i][0]);
		path.push_back(nodes[i][1]);

		auto neighbors = getNeighbors(path, NEIGHBORHOOD_VERTEX);

		EXPECT_EQ(nodes[i][2], neighbors.size())
				<< "Incorrect Neighbor Count for Node #" << i
				<< " [" << path[0] << ", " << path[1] << "].";
	}
}

}
}

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
/// Tests for the OctreeNode class.

#include "gtest/gtest.h"
#include <array>
#include <memory>
#include <string>

#include "SurgSim/DataStructures/OctreeNode.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::OctreeNode;
using SurgSim::Math::Vector3d;

struct MockData
{
	int mockInt;
	double mockDouble;
	std::string mockString;
};

typedef OctreeNode<MockData> OctreeNodeType;
typedef OctreeNode<MockData>::BoundingBoxType BoundingBoxType;

namespace
{
const double epsilon = 1e-14;
}

TEST(OctreeNodeTests, CanConstruct)
{
	BoundingBoxType boundingBox(Vector3d::Zero(), Vector3d::Ones());

	EXPECT_NO_THROW({OctreeNodeType octree(boundingBox);});
	EXPECT_NO_THROW(std::make_shared<OctreeNodeType>(boundingBox));
}

TEST(OctreeNodeTests, InitialValues)
{
	BoundingBoxType expectedBoundingBox(Vector3d::Zero(), Vector3d::Ones());
	OctreeNodeType octree(expectedBoundingBox);

	EXPECT_FALSE(octree.isActive());
	EXPECT_TRUE(octree.isLeafNode());
	EXPECT_TRUE(expectedBoundingBox.isApprox(octree.getBoundingBox()));
	EXPECT_EQ(0, octree.getChildren().size());
}

TEST(OctreeNodeTests, Subdivide)
{
	BoundingBoxType boundingBox(Vector3d::Zero(), Vector3d::Ones() * 16.0);
	OctreeNodeType octree(boundingBox);

	EXPECT_EQ(0, octree.getChildren().size());
	EXPECT_TRUE(octree.isLeafNode());
	EXPECT_FALSE(octree.isActive());
	octree.subdivide();
	EXPECT_EQ(8, octree.getChildren().size());
	EXPECT_FALSE(octree.isLeafNode());
	EXPECT_FALSE(octree.isActive());

	auto children = octree.getChildren();
	for(auto child=children.cbegin(); child!=children.cend(); ++child)
	{
		EXPECT_FALSE((*child)->isActive());
		EXPECT_TRUE((*child)->isLeafNode());
		EXPECT_EQ(0, (*child)->getChildren().size());
	}

	std::array<BoundingBoxType, 8> expectedBoxes = {
		BoundingBoxType(Vector3d(0.0, 0.0, 0.0), Vector3d(8.0, 8.0, 8.0)),
		BoundingBoxType(Vector3d(0.0, 0.0, 8.0), Vector3d(8.0, 8.0, 16.0)),
		BoundingBoxType(Vector3d(0.0, 8.0, 0.0), Vector3d(8.0, 16.0, 8.0)),
		BoundingBoxType(Vector3d(0.0, 8.0, 8.0), Vector3d(8.0, 16.0, 16.0)),
		BoundingBoxType(Vector3d(8.0, 0.0, 0.0), Vector3d(16.0, 8.0, 8.0)),
		BoundingBoxType(Vector3d(8.0, 0.0, 8.0), Vector3d(16.0, 8.0, 16.0)),
		BoundingBoxType(Vector3d(8.0, 8.0, 0.0), Vector3d(16.0, 16.0, 8.0)),
		BoundingBoxType(Vector3d(8.0, 8.0, 8.0), Vector3d(16.0, 16.0, 16.0))};
	for (auto expectedBox=expectedBoxes.cbegin(); expectedBox!=expectedBoxes.cend(); ++expectedBox)
	{
		bool boxFound = false;
		for(auto child=children.cbegin(); child!=children.cend(); ++child)
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
	auto children = node->getChildren();
	for(auto child=children.cbegin(); child!=children.cend(); ++child)
	{
		if ((*child)->isActive())
		{
			return countOctreeLevels(*child) + 1;
		}
	}
	return 1;
}

TEST(OctreeNodeTests, AddNodes)
{
	BoundingBoxType boundingBox(Vector3d::Ones() * (-8.0), Vector3d::Ones() * 8.0);
	std::shared_ptr<OctreeNodeType> octree = std::make_shared<OctreeNodeType>(boundingBox);

	const int levels = 5;
	MockData data = {1, 3.14, "string"};

	EXPECT_EQ(0, octree->getChildren().size());
	EXPECT_TRUE(octree->isLeafNode());
	EXPECT_FALSE(octree->isActive());

	EXPECT_TRUE(octree->addData(Vector3d(1.0, 1.0, 1.0), data, levels));
	EXPECT_TRUE(octree->addData(Vector3d(-4.0, 5.0, -7.0), data, levels));

	EXPECT_EQ(8, octree->getChildren().size());
	EXPECT_FALSE(octree->isLeafNode());
	EXPECT_TRUE(octree->isActive());
	EXPECT_EQ(5, countOctreeLevels(octree));

	int numActive = 0;
	auto children = octree->getChildren();
	for(auto child=children.cbegin(); child!=children.cend(); ++child)
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
	BoundingBoxType boundingBox(Vector3d::Ones() * (-8.0), Vector3d::Ones() * 8.0);
	OctreeNodeType octree(boundingBox);

	const int levels = 1;
	MockData expectedData = {1, 3.14, "string"};

	EXPECT_EQ(0, octree.getChildren().size());
	EXPECT_TRUE(octree.isLeafNode());
	EXPECT_FALSE(octree.isActive());
	EXPECT_TRUE(octree.addData(Vector3d(1.0, 1.0, 1.0), expectedData, levels));
	EXPECT_EQ(0, octree.getChildren().size());
	EXPECT_TRUE(octree.isLeafNode());
	EXPECT_TRUE(octree.isActive());

	EXPECT_EQ(expectedData.mockInt, octree.data.mockInt);
	EXPECT_NEAR(expectedData.mockDouble, octree.data.mockDouble, epsilon);
	EXPECT_EQ(expectedData.mockString, octree.data.mockString);
}

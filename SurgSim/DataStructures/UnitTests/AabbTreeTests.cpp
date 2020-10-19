#include "../PlyReader.h"
#include "../TriangleMeshPlyReaderDelegate.h"
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
#include "SurgSim/DataStructures/AabbTreeIntersectionVisitor.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Timer.h"
#include "SurgSim/Math/Aabb.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Testing/MathUtilities.h"

using SurgSim::Math::Aabbd;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace DataStructures
{

TEST(AabbTreeTests, InitTest)
{
	ASSERT_NO_THROW({AabbTree tree(3);});

	auto tree = std::make_shared<AabbTree>(3);

	EXPECT_EQ(3u, tree->getMaxObjectsPerNode());
	EXPECT_NE(nullptr, tree->getRoot());
}

TEST(AabbTreeTests, AddTest)
{
	auto tree = std::make_shared<AabbTree>(3);

	Aabbd one(Vector3d(-1.0, -1.0, -1.0), Vector3d(0.0, 0.0, 0.0));
	Aabbd two(Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));

	EXPECT_NO_THROW(tree->add(one, 0));
	EXPECT_NO_THROW(tree->add(one, 1));
	EXPECT_NO_THROW(tree->add(two, 2));
	EXPECT_NO_THROW(tree->add(two, 3));

	EXPECT_EQ(2u, tree->getRoot()->getNumChildren());
}

TEST(AabbTreeTests, UpdateTest)
{
	auto tree = std::make_shared<AabbTree>(3);

	Aabbd bigBox;
	for (int i = 0; i <= 6; ++i)
	{
		Aabbd aabb(Vector3d(static_cast<double>(i) - 0.01, -0.01, -0.01),
				   Vector3d(static_cast<double>(i) + 0.01, 0.01, 0.01));
		bigBox.extend(aabb);
		tree->add(aabb, i);
	}

	EXPECT_TRUE(bigBox.isApprox(tree->getAabb())) << bigBox << ", " << tree->getAabb();

	std::vector<Aabbd> newBoxes;

	bigBox.setEmpty();
	for (int i = 0; i <= 6; ++i)
	{
		Aabbd aabb(Vector3d(static_cast<double>(i) - 0.01, -0.02, -0.02),
				   Vector3d(static_cast<double>(i) + 0.01, 0.02, 0.02));
		bigBox.extend(aabb);
		newBoxes.push_back(aabb);
	}

	tree->updateBounds(newBoxes);

	EXPECT_TRUE(bigBox.isApprox(tree->getAabb())) << bigBox << ", " << tree->getAabb();

}


TEST(AabbTreeTests, BuildTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	auto tree = std::make_shared<AabbTree>(3);

	auto mesh = std::make_shared<TriangleMeshPlain>();
	mesh->load("Geometry/arm_collision.ply");

	for (size_t i = 0; i < mesh->getNumTriangles(); ++i)
	{
		auto triangle = mesh->getTriangle(i);
		Aabbd aabb(SurgSim::Math::makeAabb(
					   mesh->getVertex(triangle.verticesId[0]).position,
					   mesh->getVertex(triangle.verticesId[1]).position,
					   mesh->getVertex(triangle.verticesId[2]).position));
		tree->add(std::move(aabb), i);
	}
}

TEST(AabbTreeTests, BatchBuildTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	auto tree = std::make_shared<AabbTree>(3);

	auto mesh = std::make_shared<TriangleMeshPlain>();
	mesh->load("Geometry/arm_collision.ply");

	std::list<AabbTreeData::Item> items;
	for (size_t i = 0; i < mesh->getNumTriangles(); ++i)
	{
		auto triangle = mesh->getTriangle(i);
		Aabbd aabb(SurgSim::Math::makeAabb(
					   mesh->getVertex(triangle.verticesId[0]).position,
					   mesh->getVertex(triangle.verticesId[1]).position,
					   mesh->getVertex(triangle.verticesId[2]).position));
		items.emplace_back(std::make_pair(std::move(aabb), i));
	}
	tree->set(std::move(items));
}

TEST(AabbTreeTests, EasyIntersectionTest)
{
	auto tree = std::make_shared<AabbTree>(3);

	Aabbd bigBox;
	for (int i = 0; i <= 6; ++i)
	{
		Aabbd aabb(Vector3d(static_cast<double>(i) - 0.01, -0.01, -0.01),
				   Vector3d(static_cast<double>(i) + 0.01, 0.01, 0.01));
		bigBox.extend(aabb);
		tree->add(aabb, i);
	}

	EXPECT_TRUE(bigBox.isApprox(tree->getAabb())) << bigBox << ", " << tree->getAabb();
	AabbTreeIntersectionVisitor visitor(bigBox);
	tree->getRoot()->accept(&visitor);

	EXPECT_EQ(7u, visitor.getIntersections().size());

	Aabbd leftBox(Vector3d(0.0, -0.02, -0.02), Vector3d(3.4, 0.02, 0.02));
	visitor.setAabb(leftBox);
	tree->getRoot()->accept(&visitor);
	EXPECT_EQ(4u, visitor.getIntersections().size()) << "Left Box Incorrect";

	Aabbd middleBox(Vector3d(1.8, -0.02, -0.02), Vector3d(4.4, 0.02, 0.02));
	visitor.setAabb(middleBox);
	tree->getRoot()->accept(&visitor);
	EXPECT_EQ(3u, visitor.getIntersections().size()) << "Middle Box Incorrect";

	Aabbd rightBox(Vector3d(2.8, -0.02, -0.02), Vector3d(6.4, 0.02, 0.02));
	visitor.setAabb(rightBox);
	tree->getRoot()->accept(&visitor);
	EXPECT_EQ(4u, visitor.getIntersections().size()) << "Right Box Incorrect";

}

TEST(AabbTreeTests, MeshIntersectionTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	auto tree = std::make_shared<AabbTree>(3);

	auto mesh = std::make_shared<TriangleMeshPlain>();
	mesh->load("Geometry/arm_collision.ply");

	Aabbd expectedBigBox;

	for (size_t i = 0; i < mesh->getNumTriangles(); ++i)
	{
		auto triangle = mesh->getTriangle(i);
		std::array<Vector3d, 3> vertices =
		{
			mesh->getVertex(triangle.verticesId[0]).position,
			mesh->getVertex(triangle.verticesId[1]).position,
			mesh->getVertex(triangle.verticesId[2]).position
		};
		Aabbd aabb(SurgSim::Math::makeAabb(vertices[0], vertices[1], vertices[2]));
		expectedBigBox.extend(aabb);
		tree->add(std::move(aabb), i);
	}

	Aabbd bigBox = tree->getAabb();

	EXPECT_TRUE(expectedBigBox.isApprox(bigBox));

	AabbTreeIntersectionVisitor intersector(bigBox);

	SurgSim::Framework::Timer timer;
	timer.start();
	tree->getRoot()->accept(&intersector);
	timer.endFrame();

	EXPECT_EQ(mesh->getNumTriangles(), intersector.getIntersections().size());
}

template <typename NodeType>
class TreeLeavesVisitor : public TreeVisitor
{
public:

	bool handle(TreeNode* node) override
	{
		SURGSIM_FAILURE() << "Function " << __FUNCTION__ << " not implemented";
		return false;
	}

	bool handle(NodeType* node) override
	{
		if (node->getNumChildren() == 0)
		{
			leaves.push_back(node);
		}
		return true;
	}

	std::vector<NodeType*> leaves;
};

template <typename PairTypeLhs, typename PairTypeRhs>
static typename std::vector<PairTypeLhs>::const_iterator getEquivalentPair(const std::vector<PairTypeLhs>& list,
		const PairTypeRhs& item)
{
	return std::find_if(list.cbegin(), list.cend(),
						[&item](const PairTypeLhs & pair)
	{
		return (pair.first->getAabb().isApprox(item.first->getAabb())
				&& pair.second->getAabb().isApprox(item.second->getAabb()))
			   || (pair.first->getAabb().isApprox(item.second->getAabb())
				   && pair.second->getAabb().isApprox(item.first->getAabb()));
	}
					   );
}

TEST(AabbTreeTests, SpatialJoinTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	const std::string fileName = "Geometry/staple_collision.ply";

	auto meshA = std::make_shared<SurgSim::Math::MeshShape>();
	ASSERT_NO_THROW(meshA->load(fileName));

	auto meshB = std::make_shared<SurgSim::Math::MeshShape>();
	ASSERT_NO_THROW(meshB->load(fileName));

	RigidTransform3d rhsPose = SurgSim::Math::makeRigidTranslation(Vector3d(0.005, 0.0, 0.0));
	meshB->transform(rhsPose);

	// update the AABB trees
	meshA->update();
	meshB->update();

	auto aabbA = meshA->getAabbTree();
	auto aabbB = meshB->getAabbTree();

	auto actualIntersection = aabbA->spatialJoin(*aabbB);

	TreeLeavesVisitor<SurgSim::DataStructures::AabbTreeNode> leavesVisitorA;
	std::static_pointer_cast<SurgSim::DataStructures::AabbTreeNode>(aabbA->getRoot())->accept(&leavesVisitorA);
	auto& leavesA = leavesVisitorA.leaves;

	TreeLeavesVisitor<SurgSim::DataStructures::AabbTreeNode> leavesVisitorB;
	std::static_pointer_cast<SurgSim::DataStructures::AabbTreeNode>(aabbB->getRoot())->accept(&leavesVisitorB);
	auto& leavesB = leavesVisitorB.leaves;

	std::vector<std::pair<SurgSim::DataStructures::AabbTreeNode*, SurgSim::DataStructures::AabbTreeNode*>>
			expectedIntersection;
	for (auto leafA = leavesA.begin(); leafA != leavesA.end(); ++leafA)
	{
		for (auto leafB = leavesB.begin(); leafB != leavesB.end(); ++leafB)
		{
			if (SurgSim::Math::doAabbIntersect((*leafA)->getAabb(), (*leafB)->getAabb()))
			{
				expectedIntersection.emplace_back(*leafA, *leafB);
			}
		}
	}

	{
		SCOPED_TRACE("Equivalent sets");

		ASSERT_GT(expectedIntersection.size(), 0u);
		ASSERT_EQ(expectedIntersection.size(), actualIntersection.size());

		// Sets A and B are equal if and only if A is a subset of B and B is a subset of A.
		for (auto it = actualIntersection.begin(); it != actualIntersection.end(); ++it)
		{
			EXPECT_FALSE(getEquivalentPair(expectedIntersection, *it) == expectedIntersection.cend());
		}

		for (auto it = expectedIntersection.begin(); it != expectedIntersection.end(); ++it)
		{
			EXPECT_FALSE(getEquivalentPair(actualIntersection, *it) == actualIntersection.cend());
		}
	}

	{
		SCOPED_TRACE("Inequivalent sets.");

		auto newNode = std::make_shared<SurgSim::DataStructures::AabbTreeNode>();
		newNode->addData(
			SurgSim::Math::makeAabb(Vector3d(-0.1, 0.3, 5.3), Vector3d(5.4, -5.8, 1.1), Vector3d(0, 0.5, 11)), 4543);

		expectedIntersection.emplace_back(newNode.get(), expectedIntersection.front().second);
		actualIntersection.emplace_back(newNode.get(), actualIntersection.back().first);

		ASSERT_GT(expectedIntersection.size(), 0u);
		ASSERT_EQ(expectedIntersection.size(), actualIntersection.size());

		EXPECT_TRUE(getEquivalentPair(expectedIntersection, actualIntersection.back()) == expectedIntersection.cend());
		EXPECT_TRUE(getEquivalentPair(actualIntersection, expectedIntersection.back()) == actualIntersection.cend());
	}
}

} // namespace DataStructure
} // namespace SurgSim

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

#include "SurgSim/DataStructures/AabbTree2.h"

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

TEST(AabbTreeTests, BatchBuildTestAabbTree2)
{

	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	auto tree = std::make_shared<SurgSim::Experimental::AabbTree>();

	auto mesh = std::make_shared<TriangleMeshPlain>();
	mesh->load("Geometry/arm_collision.ply");

	std::vector<Math::Aabbd> aabbs;
	std::vector<size_t> indices;
	aabbs.reserve(mesh->getNumTriangles());
	indices.reserve(mesh->getNumTriangles());
	for (size_t i = 0; i < mesh->getNumTriangles(); ++i)
	{
		auto triangle = mesh->getTriangle(i);
		Aabbd aabb(SurgSim::Math::makeAabb(
			mesh->getVertex(triangle.verticesId[0]).position,
			mesh->getVertex(triangle.verticesId[1]).position,
			mesh->getVertex(triangle.verticesId[2]).position));
		aabbs.emplace_back(std::move(aabb));
		indices.push_back(i);
	}
	tree->build(aabbs, &indices);

	std::vector<Aabbd> bounds;
	bounds.reserve(mesh->getNumTriangles());
	for (size_t i = 0; i < mesh->getNumTriangles(); ++i)
	{
		auto triangle = mesh->getTriangle(i);
		Aabbd aabb(SurgSim::Math::makeAabb(
			mesh->getVertex(triangle.verticesId[0]).position,
			mesh->getVertex(triangle.verticesId[1]).position,
			mesh->getVertex(triangle.verticesId[2]).position));
		bounds.push_back(std::move(aabb));
	}
	tree->update(bounds);
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


std::vector<std::pair<size_t, size_t>> getPairWiseIntersections(
	const std::shared_ptr<SurgSim::Math::MeshShape>& meshA,
	const std::shared_ptr<SurgSim::Math::MeshShape>& meshB)
{
	std::vector<std::pair<size_t, size_t>> intersections;
	std::vector<Math::Aabbd> leftAabb;
	std::vector<Math::Aabbd> rightAabb;

	for (size_t i = 0; i < meshA->getNumTriangles(); ++i)
	{
		auto& triangle = meshA->getTriangle(i);
		Aabbd aabb(SurgSim::Math::makeAabb(
			meshA->getVertex(triangle.verticesId[0]).position,
			meshA->getVertex(triangle.verticesId[1]).position,
			meshA->getVertex(triangle.verticesId[2]).position));
		leftAabb.push_back(aabb);
	}

	for (size_t i = 0; i < meshB->getNumTriangles(); ++i)
	{
		auto& triangle = meshB->getTriangle(i);
		Aabbd aabb(SurgSim::Math::makeAabb(
			meshB->getVertex(triangle.verticesId[0]).position,
			meshB->getVertex(triangle.verticesId[1]).position,
			meshB->getVertex(triangle.verticesId[2]).position));
		rightAabb.push_back(aabb);
	}

	for (size_t i = 0; i < leftAabb.size(); ++i)
	{
		for (size_t j = 0; j < rightAabb.size(); ++j)
		{
			if (leftAabb[i].intersects(rightAabb[j]))
			{
				intersections.emplace_back(i, j);
			}
		}
	}

	std::sort(std::begin(intersections), std::end(intersections));
	return intersections;
}

TEST(AabbTreeTests, SpatialJoinTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	//const std::string fileName = "Geometry/staple_collision.ply";
	const std::string fileName = "Geometry/arm_collision.ply";

	auto meshA = std::make_shared<SurgSim::Math::MeshShape>();
	ASSERT_NO_THROW(meshA->load(fileName));

	auto meshB = std::make_shared<SurgSim::Math::MeshShape>();
	ASSERT_NO_THROW(meshB->load(fileName));

	RigidTransform3d rhsPose = SurgSim::Math::makeRigidTranslation(Vector3d(0.05, 0.05, 0.0));
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

	std::vector<std::pair<size_t, size_t>> triIntersections;

	for (auto leafA = leavesA.begin(); leafA != leavesA.end(); ++leafA)
	{
		for (auto leafB = leavesB.begin(); leafB != leavesB.end(); ++leafB)
		{
			if (SurgSim::Math::doAabbIntersect((*leafA)->getAabb(), (*leafB)->getAabb()))
			{
				expectedIntersection.emplace_back(*leafA, *leafB);

				auto& leftData = (static_cast<AabbTreeData*>((*leafA)->getData().get())->getData());
				auto& rightData = (static_cast<AabbTreeData*>((*leafB)->getData().get())->getData());

				for (auto& leftTri : leftData)
				{
					for (auto& rightTri : rightData)
					{
						if (leftTri.first.intersects(rightTri.first))
						{
							triIntersections.emplace_back(leftTri.second, rightTri.second);
						}
					}
				}
			}
		}
	}

	std::sort(std::begin(triIntersections), std::end(triIntersections));
	ASSERT_EQ(triIntersections, getPairWiseIntersections(meshA, meshB));

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

static void fillTree(std::shared_ptr<SurgSim::Experimental::AabbTree> tree,
	std::shared_ptr<SurgSim::Math::MeshShape> mesh)
{

	std::vector<Aabbd> aabbs;
	std::vector<size_t> indices;
	aabbs.reserve(mesh->getNumTriangles());
	indices.reserve(mesh->getNumTriangles());

	for (size_t i = 0; i < mesh->getNumTriangles(); ++i)
	{
		auto triangle = mesh->getTriangle(i);
		Aabbd aabb(SurgSim::Math::makeAabb(
			mesh->getVertex(triangle.verticesId[0]).position,
			mesh->getVertex(triangle.verticesId[1]).position,
			mesh->getVertex(triangle.verticesId[2]).position));
		aabbs.emplace_back(std::move(aabb));
		indices.push_back(i);
	}
	tree->build(aabbs, &indices);
	//tree->update(aabbs);
}

TEST(AabbTree2Tests, NodeTest)
{
	using SurgSim::Math::Vector3d;
	Vector3d a(-1, 1, 1);
	Vector3d b(0, 0.5, 0.5);
	Vector3d c(1, 0, 0);

	std::vector<size_t> indices{ 0, 1 };
	std::vector<Aabbd> aabbs;

	aabbs.push_back(Aabbd().extend(a).extend(b));
	aabbs.push_back(Aabbd().extend(b).extend(c));

	Experimental::AabbTree tree;
	tree.build(aabbs, &indices);

}

TEST(AabbTree2Tests, Consistency)
{

	auto tree = std::make_shared<SurgSim::Experimental::AabbTree>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	const std::string fileName = "Geometry/arm_collision.ply";

	auto mesh = std::make_shared<SurgSim::Math::MeshShape>();
	ASSERT_NO_THROW(mesh->load(fileName));

	fillTree(tree, mesh);

	auto data = tree->getTreeData();

	std::vector<size_t> children;
	size_t index = 0;
	for (auto& node : data)
	{
		if (node.left != -1)
		{
			EXPECT_TRUE(index < node.left && index < node.right) << index;
			children.push_back(node.left);
			children.push_back(node.right);

			Aabbd parent(node.aabb);
			Aabbd left(data[node.left].aabb);
			Aabbd right(data[node.right].aabb);
			Aabbd combined(left);
			combined.extend(right);
			EXPECT_TRUE(node.aabb.isApprox(combined)) << index;
		}
		++index;
	}

	std::sort(std::begin(children), std::end(children));

	for (int i = 0; i < children.size() - 1; ++i)
	{
		EXPECT_TRUE(children[i + 1] - children[i] == 1);
	}
}

TEST(AabbTree2Tests, SpatialJoinTestAabbTree2)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	//const std::string fileName = "Geometry/staple_collision.ply";
	const std::string fileName = "Geometry/arm_collision.ply";

	auto meshA = std::make_shared<SurgSim::Math::MeshShape>();
	ASSERT_NO_THROW(meshA->load(fileName));

	auto meshB = std::make_shared<SurgSim::Math::MeshShape>();
	ASSERT_NO_THROW(meshB->load(fileName));

	RigidTransform3d rhsPose = SurgSim::Math::makeRigidTranslation(Vector3d(0.05, 0.05, 0.0));
	meshB->transform(rhsPose);

	// update the AABB trees
	meshA->update();
	meshB->update();

	std::vector<std::pair<size_t, size_t>> intersections;

	auto treeA = std::make_shared<SurgSim::Experimental::AabbTree>();
	fillTree(treeA, meshA);

	auto treeB = std::make_shared<SurgSim::Experimental::AabbTree>();
	fillTree(treeB, meshB);

	treeA->recursiveSpatialJoin(*treeB, &intersections, 0, 0);
	//treeA->spatialJoin(*treeB, &intersections);
	std::sort(std::begin(intersections), std::end(intersections));

	auto result = getPairWiseIntersections(meshA, meshB);

	EXPECT_EQ(result, intersections);

}

} // namespace DataStructure
} // namespace SurgSim

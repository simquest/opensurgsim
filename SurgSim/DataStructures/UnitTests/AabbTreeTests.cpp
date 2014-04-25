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
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/DataStructures/AabbTreeIntersectionVisitor.h"
#include "SurgSim/Math/Aabb.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Timer.h"
#include "SurgSim/Testing/MathUtilities.h"

using SurgSim::Math::Aabbd;
using SurgSim::Math::Vector3d;

namespace
{
std::shared_ptr<SurgSim::DataStructures::TriangleMeshBase<
SurgSim::DataStructures::EmptyData, SurgSim::DataStructures::EmptyData, SurgSim::DataStructures::EmptyData>>
		loadTriangleMesh(const std::string& fileName)
{
	auto triangleMeshDelegate = std::make_shared<SurgSim::DataStructures::TriangleMeshPlyReaderDelegate>();

	SurgSim::DataStructures::PlyReader reader(fileName);
	SURGSIM_ASSERT(reader.setDelegate(triangleMeshDelegate)) << "The input file " << fileName << " is malformed.";
	reader.parseFile();

	return triangleMeshDelegate->getMesh();
}
}

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

TEST(AabbTreeTests, BuildTest)
{
	SurgSim::Framework::ApplicationData data("config.txt");
	auto tree = std::make_shared<AabbTree>(3);

	std::string filename = data.findFile("Geometry/arm_collision.ply");
	ASSERT_FALSE(filename.empty());
	auto mesh = loadTriangleMesh(filename);

	std::cout << mesh->getNumTriangles() << std::endl;


	SurgSim::Framework::Timer timer;
	for (size_t i = 0; i < mesh->getNumTriangles(); ++i)
	{
		auto triangle = mesh->getTriangle(i);
		Aabbd aabb(SurgSim::Math::makeAabb(
					   mesh->getVertex(triangle.verticesId[0]).position,
					   mesh->getVertex(triangle.verticesId[1]).position,
					   mesh->getVertex(triangle.verticesId[2]).position));
		tree->add(aabb, i);
	}
	timer.endFrame();
	std::cout << timer.getCumulativeTime() << std::endl;
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
	SurgSim::Framework::ApplicationData data("config.txt");
	auto tree = std::make_shared<AabbTree>(3);

	std::string filename = data.findFile("Geometry/arm_collision.ply");
	ASSERT_FALSE(filename.empty());
	auto mesh = loadTriangleMesh(filename);

	std::cout << mesh->getNumTriangles() << std::endl;

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

	std::cout << timer.getCumulativeTime() << std::endl;
	std::cout << intersector.getIntersections().size() << std::endl;

	EXPECT_EQ(mesh->getNumTriangles(), intersector.getIntersections().size());
}



}
}


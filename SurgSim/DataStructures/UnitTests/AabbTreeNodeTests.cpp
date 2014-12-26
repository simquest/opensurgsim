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

#include "SurgSim/Math/Aabb.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/DataStructures/TreeVisitor.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"

using SurgSim::Math::Aabbd;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace DataStructures
{

TEST(AabbTreeNodeTests, InitTest)
{
	EXPECT_NO_THROW(AabbTreeNode node);

	auto node = std::make_shared<AabbTreeNode>();
	EXPECT_EQ(0u, node->getNumChildren());
	EXPECT_TRUE(node->getAabb().isEmpty());
}

TEST(AabbTreeNodeTests, AddTest)
{
	auto node = std::make_shared<AabbTreeNode>();
	Aabbd one(Vector3d(-1.0, -1.0, -1.0), Vector3d(0.0, 0.0, 0.0));
	Aabbd two(Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));
	Aabbd combined(Vector3d(-1.0, -1.0, -1.0), Vector3d(1.0, 1.0, 1.0));
	EXPECT_NO_THROW(node->addData(one, 0));

	EXPECT_EQ(0u, node->getNumChildren());
	EXPECT_TRUE(one.isApprox(node->getAabb()));

	node->addData(one, 1, 3);
	node->addData(two, 2, 3);
	node->addData(two, 3, 3);

	ASSERT_EQ(2u, node->getNumChildren());
	EXPECT_TRUE(combined.isApprox(node->getAabb()));

	ASSERT_NE(nullptr, node->getChild(0));
	ASSERT_NE(nullptr, node->getChild(1));

	auto child = std::dynamic_pointer_cast<AabbTreeNode>(node->getChild(0));
	ASSERT_NE(nullptr, child);
	ASSERT_EQ(0u, child->getNumChildren());
	EXPECT_TRUE(one.isApprox(child->getAabb()));


	child = std::dynamic_pointer_cast<AabbTreeNode>(node->getChild(1));
	ASSERT_NE(nullptr, child);
	ASSERT_EQ(0u, child->getNumChildren());
	EXPECT_TRUE(two.isApprox(child->getAabb()));

}

class TestVisitor : public TreeVisitor
{
public:
	virtual ~TestVisitor() {}

	bool handle(TreeNode* node) override
	{
		throw std::logic_error("The method or operation is not implemented.");
		return false;
	}

	bool handle(AabbTreeNode* node) override
	{
		return true;
	}

};

TEST(AabbTreeNodeTests, VisitorTest)
{
	TestVisitor visitor;
	auto node = std::make_shared<AabbTreeNode>();
	node->accept(&visitor);
}


}
}


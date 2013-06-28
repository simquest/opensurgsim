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
/// Tests for the Group class.

#include <SurgSim/Graphics/UnitTests/MockObjects.h>

#include <gtest/gtest.h>

namespace SurgSim
{

namespace Graphics
{

TEST(GroupTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Group> group = std::make_shared<MockGroup>("test name");});
}

TEST(GroupTests, VisibilityTest)
{
	std::shared_ptr<Group> group = std::make_shared<MockGroup>("test name");

	group->setVisible(true);
	EXPECT_TRUE(group->isVisible());

	group->setVisible(false);
	EXPECT_FALSE(group->isVisible());
}

TEST(GroupTests, AddRemoveTest)
{
	std::shared_ptr<Group> group = std::make_shared<MockGroup>("test name");

	std::shared_ptr<Representation> representation1 = std::make_shared<MockRepresentation>("test representation 1");
	std::shared_ptr<Representation> representation2 = std::make_shared<MockRepresentation>("test representation 2");
	std::shared_ptr<MockGroup> group1 = std::make_shared<MockGroup>("test group 1");
	std::shared_ptr<MockGroup> group2 = std::make_shared<MockGroup>("test group 2");

	EXPECT_EQ(0u, group->getMembers().size());

	/// Add an representation
	EXPECT_TRUE(group->add(representation1));
	EXPECT_EQ(1u, group->getMembers().size());
	EXPECT_NE(group->getMembers().end(), std::find(group->getMembers().begin(), group->getMembers().end(), representation1));

	/// Add another representation
	EXPECT_TRUE(group->add(representation2));
	EXPECT_EQ(2u, group->getMembers().size());
	EXPECT_NE(group->getMembers().end(), std::find(group->getMembers().begin(), group->getMembers().end(), representation2));


	/// Try to add a duplicate representation
	EXPECT_FALSE(group->add(representation1));
	EXPECT_EQ(2u, group->getMembers().size());


	/// Remove an representation
	EXPECT_TRUE(group->remove(representation1));
	EXPECT_EQ(group->getMembers().end(), std::find(group->getMembers().begin(), group->getMembers().end(), representation1));


	/// Try to remove an representation that is not in the group
	EXPECT_FALSE(group->remove(representation1));
	EXPECT_EQ(group->getMembers().end(), std::find(group->getMembers().begin(), group->getMembers().end(), representation1));
}

TEST(GroupTests, AppendTest)
{
	std::shared_ptr<Group> group1 = std::make_shared<MockGroup>("test group 1");
	EXPECT_EQ(0u, group1->getMembers().size());

	std::shared_ptr<Representation> representation1 = std::make_shared<MockRepresentation>("test representation 1");
	std::shared_ptr<Representation> representation2 = std::make_shared<MockRepresentation>("test representation 2");

	/// Add 2 representations to group 1
	EXPECT_TRUE(group1->add(representation1));
	EXPECT_TRUE(group1->add(representation2));
	EXPECT_EQ(2u, group1->getMembers().size());

	std::shared_ptr<Representation> representation3 = std::make_shared<MockRepresentation>("test representation 3");

	std::shared_ptr<Group> group2 = std::make_shared<MockGroup>("test group 2");
	EXPECT_EQ(0u, group2->getMembers().size());

	/// Add an representation to group 2 and append group 1 to group 2.
	EXPECT_TRUE(group2->add(representation3));
	EXPECT_TRUE(group2->append(group1));
	EXPECT_EQ(3u, group2->getMembers().size());

	/// Check that the representations from group 1 were added to group 2, and that it still has the representation that was added
	/// directly to it.
	EXPECT_NE(group2->getMembers().end(), std::find(group2->getMembers().begin(), group2->getMembers().end(), representation1));
	EXPECT_NE(group2->getMembers().end(), std::find(group2->getMembers().begin(), group2->getMembers().end(), representation2));
	EXPECT_NE(group2->getMembers().end(), std::find(group2->getMembers().begin(), group2->getMembers().end(), representation3));

	/// Try to append a group that has already been appended - this will try to add duplicate representations.
	EXPECT_FALSE(group2->append(group1)) << "Append should return false if any representation is a duplicate!";
	EXPECT_EQ(3u, group2->getMembers().size());

	/// Check that group 1 was not modified by appending it to group 2.
	EXPECT_EQ(2u, group1->getMembers().size());
	EXPECT_NE(group1->getMembers().end(), std::find(group1->getMembers().begin(), group1->getMembers().end(), representation1));
	EXPECT_NE(group1->getMembers().end(), std::find(group1->getMembers().begin(), group1->getMembers().end(), representation2));
}

TEST(GroupTests, ClearTest)
{
	std::shared_ptr<Group> group = std::make_shared<MockGroup>("test name");

	std::shared_ptr<Representation> representation1 = std::make_shared<MockRepresentation>("test representation 1");
	std::shared_ptr<Representation> representation2 = std::make_shared<MockRepresentation>("test representation 2");
	std::shared_ptr<Representation> representation3 = std::make_shared<MockRepresentation>("test representation 3");

	EXPECT_EQ(0u, group->getMembers().size());

	/// Add 3 representations
	EXPECT_TRUE(group->add(representation1));
	EXPECT_TRUE(group->add(representation2));
	EXPECT_TRUE(group->add(representation3));
	EXPECT_EQ(3u, group->getMembers().size());

	/// Remove all representations and make sure that they are removed correctly
	group->clear();
	EXPECT_EQ(0u, group->getMembers().size());
}

};  // namespace Graphics

};  // namespace SurgSim

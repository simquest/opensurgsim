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

#include "SurgSim/Graphics/UnitTests/MockObjects.h"

#include "gtest/gtest.h"

using SurgSim::Graphics::Actor;
using SurgSim::Graphics::Group;

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

	std::shared_ptr<Actor> actor1 = std::make_shared<MockActor>("test actor 1");
	std::shared_ptr<Actor> actor2 = std::make_shared<MockActor>("test actor 2");
	std::shared_ptr<MockGroup> group1 = std::make_shared<MockGroup>("test group 1");
	std::shared_ptr<MockGroup> group2 = std::make_shared<MockGroup>("test group 2");

	EXPECT_EQ(0, group->getActors().size());
	EXPECT_EQ(0, group->getGroups().size());

	/// Add an actor
	EXPECT_TRUE(group->addActor(actor1));
	EXPECT_EQ(1, group->getActors().size());
	EXPECT_NE(group->getActors().end(), std::find(group->getActors().begin(), 
		group->getActors().end(), actor1));

	/// Add a group
	EXPECT_TRUE(group->addGroup(group1));
	EXPECT_EQ(1, group->getGroups().size());
	EXPECT_NE(group->getGroups().end(), std::find(group->getGroups().begin(), 
		group->getGroups().end(), group1));

	/// Add another group
	EXPECT_TRUE(group->addGroup(group2));
	EXPECT_EQ(2, group->getGroups().size());
	EXPECT_NE(group->getGroups().end(), std::find(group->getGroups().begin(), 
		group->getGroups().end(), group2));

	/// Add another actor
	EXPECT_TRUE(group->addActor(actor2));
	EXPECT_EQ(2, group->getActors().size());
	EXPECT_NE(group->getActors().end(), std::find(group->getActors().begin(), 
		group->getActors().end(), actor2));


	/// Try to add a duplicate actor
	EXPECT_FALSE(group->addActor(actor1));
	EXPECT_EQ(2, group->getActors().size());

	/// Try to add a duplicate group
	EXPECT_FALSE(group->addGroup(group2));
	EXPECT_EQ(2, group->getGroups().size());


	/// Remove a group
	EXPECT_TRUE(group->removeGroup(group2));
	EXPECT_EQ(group->getGroups().end(), std::find(group->getGroups().begin(), 
		group->getGroups().end(), group2));

	/// Remove an actor
	EXPECT_TRUE(group->removeActor(actor1));
	EXPECT_EQ(group->getActors().end(), std::find(group->getActors().begin(), 
		group->getActors().end(), actor1));

	/// Try to remove a group that is not in the group
	EXPECT_FALSE(group->removeGroup(group2));
	EXPECT_EQ(group->getGroups().end(), std::find(group->getGroups().begin(), 
		group->getGroups().end(), group2));

	/// Try to remove an actor that is not in the group
	EXPECT_FALSE(group->removeActor(actor1));
	EXPECT_EQ(group->getActors().end(), std::find(group->getActors().begin(), 
		group->getActors().end(), actor1));
}


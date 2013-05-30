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

	EXPECT_EQ(0u, group->getActors().size());
	EXPECT_EQ(0u, group->getGroups().size());

	/// Add an actor
	EXPECT_TRUE(group->addActor(actor1));
	EXPECT_EQ(1u, group->getActors().size());
	EXPECT_NE(group->getActors().end(), std::find(group->getActors().begin(), group->getActors().end(), actor1));

	/// Add a group
	EXPECT_TRUE(group->addGroup(group1));
	EXPECT_EQ(1u, group->getGroups().size());
	EXPECT_NE(group->getGroups().end(), std::find(group->getGroups().begin(), group->getGroups().end(), group1));

	/// Add another group
	EXPECT_TRUE(group->addGroup(group2));
	EXPECT_EQ(2u, group->getGroups().size());
	EXPECT_NE(group->getGroups().end(), std::find(group->getGroups().begin(), group->getGroups().end(), group2));

	/// Add another actor
	EXPECT_TRUE(group->addActor(actor2));
	EXPECT_EQ(2u, group->getActors().size());
	EXPECT_NE(group->getActors().end(), std::find(group->getActors().begin(), group->getActors().end(), actor2));


	/// Try to add a duplicate actor
	EXPECT_FALSE(group->addActor(actor1));
	EXPECT_EQ(2u, group->getActors().size());

	/// Try to add a duplicate group
	EXPECT_FALSE(group->addGroup(group2));
	EXPECT_EQ(2u, group->getGroups().size());


	/// Remove a group
	EXPECT_TRUE(group->removeGroup(group2));
	EXPECT_EQ(group->getGroups().end(), std::find(group->getGroups().begin(), group->getGroups().end(), group2));

	/// Remove an actor
	EXPECT_TRUE(group->removeActor(actor1));
	EXPECT_EQ(group->getActors().end(), std::find(group->getActors().begin(), group->getActors().end(), actor1));

	/// Try to remove a group that is not in the group
	EXPECT_FALSE(group->removeGroup(group2));
	EXPECT_EQ(group->getGroups().end(), std::find(group->getGroups().begin(), group->getGroups().end(), group2));

	/// Try to remove an actor that is not in the group
	EXPECT_FALSE(group->removeActor(actor1));
	EXPECT_EQ(group->getActors().end(), std::find(group->getActors().begin(), group->getActors().end(), actor1));
}

TEST(GroupTests, ClearTests)
{
	std::shared_ptr<Group> group = std::make_shared<MockGroup>("test name");

	std::shared_ptr<Actor> actor1 = std::make_shared<MockActor>("test actor 1");
	std::shared_ptr<Actor> actor2 = std::make_shared<MockActor>("test actor 2");
	std::shared_ptr<Actor> actor3 = std::make_shared<MockActor>("test actor 3");
	std::shared_ptr<Group> group1 = std::make_shared<MockGroup>("test group 1");
	std::shared_ptr<Group> group2 = std::make_shared<MockGroup>("test group 2");

	EXPECT_EQ(0u, group->getActors().size());
	EXPECT_EQ(0u, group->getGroups().size());

	/// Add 3 actors and 2 groups
	group->addActor(actor1);
	group->addActor(actor2);
	group->addActor(actor3);
	group->addGroup(group1);
	group->addGroup(group2);
	EXPECT_EQ(3u, group->getActors().size());
	EXPECT_EQ(2u, group->getGroups().size());

	/// Remove all actors and make sure that they are removed correctly
	group->clearActors();
	EXPECT_EQ(0u, group->getActors().size());
	EXPECT_EQ(2u, group->getGroups().size());

	/// Add the 3 actors again
	group->addActor(actor1);
	group->addActor(actor2);
	group->addActor(actor3);
	EXPECT_EQ(3u, group->getActors().size());
	EXPECT_EQ(2u, group->getGroups().size());

	/// Remove all groups and make sure that they are removed correctly
	group->clearGroups();
	EXPECT_EQ(3u, group->getActors().size());
	EXPECT_EQ(0u, group->getGroups().size());

	/// Add the 2 groups again
	group->addGroup(group1);
	group->addGroup(group2);
	EXPECT_EQ(3u, group->getActors().size());
	EXPECT_EQ(2u, group->getGroups().size());

	/// Remove everything from the group and make sure that there are no actors nor groups remaining
	group->clear();
	EXPECT_EQ(0u, group->getActors().size());
	EXPECT_EQ(0u, group->getGroups().size());
}
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
#include <SurgSim/Graphics/UnitTests/MockOsgObjects.h>

#include <SurgSim/Graphics/OsgGroup.h>

#include <gtest/gtest.h>

namespace SurgSim
{
namespace Graphics
{

TEST(OsgGroupTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Group> group = std::make_shared<OsgGroup>("test name");});

	std::shared_ptr<OsgGroup> osgGroup = std::make_shared<OsgGroup>("test group");
	std::shared_ptr<Group> group = osgGroup;

	EXPECT_EQ("test group", group->getName());
	EXPECT_TRUE(group->isVisible());
	EXPECT_EQ(0u, group->getMembers().size());
	EXPECT_EQ(0u, osgGroup->getOsgGroup()->getNumChildren());
}

TEST(OsgGroupTests, OsgNodesTest)
{
	std::shared_ptr<OsgGroup> osgGroup = std::make_shared<OsgGroup>("test group");

	osg::ref_ptr<osg::Switch> osgSwitch = dynamic_cast<osg::Switch*>(osgGroup->getOsgGroup().get());
	ASSERT_TRUE(osgSwitch.valid()) << "Group's OSG node should be a switch!";
}

TEST(OsgGroupTests, VisibilityTest)
{
	std::shared_ptr<OsgGroup> osgGroup = std::make_shared<OsgGroup>("test group");
	std::shared_ptr<Group> group = osgGroup;

	group->setVisible(false);
	EXPECT_FALSE(group->isVisible());

	group->setVisible(true);
	EXPECT_TRUE(group->isVisible());
}

TEST(OsgGroupTests, AddRemoveTest)
{
	std::shared_ptr<OsgGroup> osgGroup = std::make_shared<OsgGroup>("test group");
	std::shared_ptr<Group> group = osgGroup;

	EXPECT_EQ(0u, group->getMembers().size());

	osg::ref_ptr<osg::Switch> osgSwitch = dynamic_cast<osg::Switch*>(osgGroup->getOsgGroup().get());
	ASSERT_TRUE(osgSwitch.valid()) << "Group's OSG node should be a switch!";
	EXPECT_EQ(0u, osgSwitch->getNumChildren());

	/// Add an actor and make sure the osg::Switch value for it is set correctly (should be true)
	std::shared_ptr<OsgActor> actor1 = std::make_shared<MockOsgActor>("test actor 1");
	group->add(actor1);
	EXPECT_EQ(1u, group->getMembers().size());
	EXPECT_EQ(1u, osgSwitch->getNumChildren());
	EXPECT_EQ(0u, osgSwitch->getChildIndex(actor1->getOsgNode()));
	EXPECT_TRUE(osgSwitch->getChildValue(actor1->getOsgNode())) << "Actor 1 should be visible!";

	/// Set group to not visible and check the osg::Switch values
	group->setVisible(false);
	EXPECT_FALSE(group->isVisible());
	EXPECT_FALSE(osgSwitch->getChildValue(actor1->getOsgNode())) << "Actor 1 should not be visible!";

	/// Add another actor and make sure the osg::Switch value for it is set correctly (should be false)
	std::shared_ptr<OsgActor> actor2 = std::make_shared<MockOsgActor>("test actor 2");
	group->add(actor2);
	EXPECT_EQ(2u, group->getMembers().size());
	EXPECT_EQ(2u, osgSwitch->getNumChildren());
	EXPECT_EQ(1u, osgSwitch->getChildIndex(actor2->getOsgNode()));
	EXPECT_FALSE(osgSwitch->getChildValue(actor2->getOsgNode())) << "Actor 2 should not be visible!";

	/// Set group to visible and check the osg::Switch values
	group->setVisible(true);
	EXPECT_TRUE(group->isVisible());
	EXPECT_TRUE(osgSwitch->getChildValue(actor1->getOsgNode())) << "Actor 1 should be visible!";
	EXPECT_TRUE(osgSwitch->getChildValue(actor2->getOsgNode())) << "Actor 2 should be visible!";

	/// Set group to not visible and check the osg::Switch values
	group->setVisible(false);
	EXPECT_FALSE(group->isVisible());
	EXPECT_FALSE(osgSwitch->getChildValue(actor1->getOsgNode())) << "Actor 1 should not be visible!";
	EXPECT_FALSE(osgSwitch->getChildValue(actor2->getOsgNode())) << "Actor 1 should not be visible!";

	/// Try to add a duplicate actor
	EXPECT_FALSE(group->add(actor1));
	EXPECT_EQ(2u, group->getMembers().size());
	EXPECT_EQ(2u, osgSwitch->getNumChildren());

	/// Remove an actor
	EXPECT_TRUE(group->remove(actor1));
	EXPECT_EQ(1u, osgSwitch->getNumChildren());
	EXPECT_EQ(1u, osgSwitch->getChildIndex(actor1->getOsgNode()));

	/// Try to remove an actor that is not in the group
	EXPECT_FALSE(group->remove(actor1));
	EXPECT_EQ(1u, osgSwitch->getChildIndex(actor1->getOsgNode()));

	/// Try to add a non-OSG actor
	std::shared_ptr<MockActor> nonOsgActor = std::make_shared<MockActor>("non-osg actor");
	EXPECT_FALSE(group->add(nonOsgActor)) << "OsgGroup should only succeed on actors that derive from OsgActor!";
	EXPECT_EQ(1u, group->getMembers().size());
	EXPECT_EQ(group->getMembers().end(), std::find(group->getMembers().begin(), group->getMembers().end(),
		nonOsgActor)) << "Only subclasses of OsgActor should be in an OsgGroup!";
	EXPECT_EQ(1u, osgSwitch->getNumChildren());
}

TEST(GroupTests, AppendTest)
{
	std::shared_ptr<Group> group1 = std::make_shared<OsgGroup>("test group 1");
	EXPECT_EQ(0u, group1->getMembers().size());

	std::shared_ptr<Actor> actor1 = std::make_shared<MockOsgActor>("test actor 1");
	std::shared_ptr<Actor> actor2 = std::make_shared<MockOsgActor>("test actor 2");

	/// Add 2 actors to group 1
	EXPECT_TRUE(group1->add(actor1));
	EXPECT_TRUE(group1->add(actor2));
	EXPECT_EQ(2u, group1->getMembers().size());

	std::shared_ptr<Actor> actor3 = std::make_shared<MockOsgActor>("test actor 3");

	std::shared_ptr<Group> group2 = std::make_shared<OsgGroup>("test group 2");
	EXPECT_EQ(0u, group2->getMembers().size());

	/// Add an actor to group 2 and append group 1 to group 2.
	EXPECT_TRUE(group2->add(actor3));
	EXPECT_TRUE(group2->append(group1));
	EXPECT_EQ(3u, group2->getMembers().size());

	/// Check that the actors from group 1 were added to group 2, and that it still has the actor that was added
	/// directly to it.
	EXPECT_NE(group2->getMembers().end(), std::find(group2->getMembers().begin(), group2->getMembers().end(), actor1));
	EXPECT_NE(group2->getMembers().end(), std::find(group2->getMembers().begin(), group2->getMembers().end(), actor2));
	EXPECT_NE(group2->getMembers().end(), std::find(group2->getMembers().begin(), group2->getMembers().end(), actor3));

	/// Try to append a group that has already been appended - this will try to add duplicate actors.
	EXPECT_FALSE(group2->append(group1)) << "Append should return false if any actor is a duplicate!";
	EXPECT_EQ(3u, group2->getMembers().size());

	/// Check that group 1 was not modified by appending it to group 2.
	EXPECT_EQ(2u, group1->getMembers().size());
	EXPECT_NE(group1->getMembers().end(), std::find(group1->getMembers().begin(), group1->getMembers().end(), actor1));
	EXPECT_NE(group1->getMembers().end(), std::find(group1->getMembers().begin(), group1->getMembers().end(), actor2));

	/// Try to append a group that is not a subclass of OsgGroup
	std::shared_ptr<Group> nonOsgGroup = std::make_shared<MockGroup>("non-osg group");
	std::shared_ptr<Actor> nonOsgActor = std::make_shared<MockActor>("non-osg actor");
	/// Add an OSG and non-OSG actor to this group.
	EXPECT_TRUE(nonOsgGroup->add(actor3));
	EXPECT_TRUE(nonOsgGroup->add(nonOsgActor));

	EXPECT_FALSE(group1->append(nonOsgGroup));
	EXPECT_EQ(2u, group1->getMembers().size()) <<
		"Nothing from the non-OSG group should have been added to the OsgGroup!";
	EXPECT_EQ(group1->getMembers().end(), std::find(group1->getMembers().begin(), group1->getMembers().end(), actor3));
	EXPECT_EQ(group1->getMembers().end(), std::find(group1->getMembers().begin(), group1->getMembers().end(),
		nonOsgActor));
}

TEST(OsgGroupTests, ClearTest)
{
	std::shared_ptr<OsgGroup> osgGroup = std::make_shared<OsgGroup>("test name");
	std::shared_ptr<Group> group = osgGroup;

	std::shared_ptr<Actor> actor1 = std::make_shared<MockOsgActor>("test actor 1");
	std::shared_ptr<Actor> actor2 = std::make_shared<MockOsgActor>("test actor 2");
	std::shared_ptr<Actor> actor3 = std::make_shared<MockOsgActor>("test actor 3");

	osg::ref_ptr<osg::Switch> osgSwitch = dynamic_cast<osg::Switch*>(osgGroup->getOsgGroup().get());
	ASSERT_TRUE(osgSwitch.valid()) << "Group's OSG node should be a switch!";

	EXPECT_EQ(0u, group->getMembers().size());
	EXPECT_EQ(0u, osgSwitch->getNumChildren());

	// Add 3 actors
	EXPECT_TRUE(group->add(actor1));
	EXPECT_TRUE(group->add(actor2));
	EXPECT_TRUE(group->add(actor3));
	EXPECT_EQ(3u, group->getMembers().size());
	EXPECT_EQ(3u, osgSwitch->getNumChildren());

	// Remove all actors
	group->clear();
	EXPECT_EQ(0u, group->getMembers().size());
	EXPECT_EQ(0u, osgSwitch->getNumChildren());
}

}  // namespace Graphics
}  // namespace SurgSim

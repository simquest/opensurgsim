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

using SurgSim::Graphics::Actor;
using SurgSim::Graphics::Group;
using SurgSim::Graphics::OsgActor;
using SurgSim::Graphics::OsgGroup;

TEST(OsgGroupTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Group> group = std::make_shared<OsgGroup>("test name");});
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

	EXPECT_TRUE(group->isVisible());

	group->setVisible(true);
	EXPECT_TRUE(group->isVisible());
}

TEST(OsgGroupTests, AddRemoveTest)
{
	std::shared_ptr<OsgGroup> osgGroup = std::make_shared<OsgGroup>("test group");
	std::shared_ptr<Group> group = osgGroup;

	EXPECT_EQ(0u, group->getActors().size());
	EXPECT_EQ(0u, group->getGroups().size());

	osg::ref_ptr<osg::Switch> osgSwitch = dynamic_cast<osg::Switch*>(osgGroup->getOsgGroup().get());
	ASSERT_TRUE(osgSwitch.valid()) << "Group's OSG node should be a switch!";
	EXPECT_EQ(0u, osgSwitch->getNumChildren());

	/// Add an actor and make sure the osg::Switch value for it is set correctly (should be true)
	std::shared_ptr<OsgActor> actor1 = std::make_shared<MockOsgActor>("test actor 1");
	group->addActor(actor1);
	EXPECT_EQ(1u, group->getActors().size());
	EXPECT_EQ(1u, osgSwitch->getNumChildren());
	EXPECT_EQ(0u, osgSwitch->getChildIndex(actor1->getOsgNode()));
	EXPECT_TRUE(osgSwitch->getChildValue(actor1->getOsgNode())) << "Actor 1 should be visible!";

	/// Set group to not visible and check the osg::Switch values
	group->setVisible(false);
	EXPECT_FALSE(group->isVisible());
	EXPECT_FALSE(osgSwitch->getChildValue(actor1->getOsgNode())) << "Actor 1 should not be visible!";

	/// Add another actor and make sure the osg::Switch value for it is set correctly (should be false)
	std::shared_ptr<OsgActor> actor2 = std::make_shared<MockOsgActor>("test actor 2");
	group->addActor(actor2);
	EXPECT_EQ(2u, group->getActors().size());
	EXPECT_EQ(2u, osgSwitch->getNumChildren());
	EXPECT_EQ(1u, osgSwitch->getChildIndex(actor2->getOsgNode()));
	EXPECT_FALSE(osgSwitch->getChildValue(actor2->getOsgNode())) << "Actor 2 should not be visible!";

	/// Set group to visible and check the osg::Switch values
	group->setVisible(true);
	EXPECT_TRUE(group->isVisible());
	EXPECT_TRUE(osgSwitch->getChildValue(actor1->getOsgNode())) << "Actor 1 should be visible!";
	EXPECT_TRUE(osgSwitch->getChildValue(actor2->getOsgNode())) << "Actor 2 should be visible!";

	/// Add a group and make sure the osg::Switch value for it is set correctly (should be true)
	std::shared_ptr<OsgGroup> subGroup1 = std::make_shared<OsgGroup>("test sub-group 1");
	group->addGroup(subGroup1);
	EXPECT_EQ(1u, group->getGroups().size());
	EXPECT_EQ(3u, osgSwitch->getNumChildren());
	EXPECT_EQ(2u, osgSwitch->getChildIndex(subGroup1->getOsgGroup()));
	EXPECT_TRUE(osgSwitch->getChildValue(subGroup1->getOsgGroup())) << "Group 1 should be visible!";

	/// Set group to not visible and check the osg::Switch values
	group->setVisible(false);
	EXPECT_FALSE(group->isVisible());
	EXPECT_FALSE(osgSwitch->getChildValue(actor1->getOsgNode())) << "Actor 1 should not be visible!";
	EXPECT_FALSE(osgSwitch->getChildValue(actor2->getOsgNode())) << "Actor 1 should not be visible!";
	EXPECT_FALSE(osgSwitch->getChildValue(subGroup1->getOsgGroup())) << "Group 1 should not be visible!";

	/// Add another group and make sure the osg::Switch value for it is set correctly (should be false)
	std::shared_ptr<OsgGroup> subGroup2 = std::make_shared<OsgGroup>("test sub-group 1");
	group->addGroup(subGroup2);
	EXPECT_EQ(2u, group->getGroups().size());
	EXPECT_EQ(4u, osgSwitch->getNumChildren());
	EXPECT_EQ(3u, osgSwitch->getChildIndex(subGroup2->getOsgGroup()));
	EXPECT_FALSE(osgSwitch->getChildValue(subGroup2->getOsgGroup())) << "Group 2 should not be visible!";

	/// Set group to visible and check the osg::Switch values
	group->setVisible(true);
	EXPECT_TRUE(group->isVisible());
	EXPECT_TRUE(osgSwitch->getChildValue(actor1->getOsgNode())) << "Actor 1 should be visible!";
	EXPECT_TRUE(osgSwitch->getChildValue(actor2->getOsgNode())) << "Actor 2 should be visible!";
	EXPECT_TRUE(osgSwitch->getChildValue(subGroup1->getOsgGroup())) << "Group 1 should be visible!";
	EXPECT_TRUE(osgSwitch->getChildValue(subGroup2->getOsgGroup())) << "Group 2 should be visible!";

	/// Try to add a duplicate actor
	EXPECT_FALSE(group->addActor(actor1));
	EXPECT_EQ(2u, group->getActors().size());
	EXPECT_EQ(4u, osgSwitch->getNumChildren());

	/// Try to add a duplicate group
	EXPECT_FALSE(group->addGroup(subGroup2));
	EXPECT_EQ(2u, group->getGroups().size());
	EXPECT_EQ(4u, osgSwitch->getNumChildren());

	/// Remove a group
	EXPECT_TRUE(group->removeGroup(subGroup2));
	EXPECT_EQ(3u, osgSwitch->getNumChildren());
	EXPECT_EQ(3u, osgSwitch->getChildIndex(subGroup2->getOsgGroup()));

	/// Remove an actor
	EXPECT_TRUE(group->removeActor(actor1));
	EXPECT_EQ(2u, osgSwitch->getNumChildren());
	EXPECT_EQ(2u, osgSwitch->getChildIndex(actor1->getOsgNode()));

	/// Try to remove a group that is not in the group
	EXPECT_FALSE(group->removeGroup(subGroup2));
	EXPECT_EQ(2u, osgSwitch->getChildIndex(subGroup2->getOsgGroup()));

	/// Try to remove an actor that is not in the group
	EXPECT_FALSE(group->removeActor(actor1));
	EXPECT_EQ(2u, osgSwitch->getChildIndex(actor1->getOsgNode()));

	/// Try to add a non-OSG actor
	std::shared_ptr<MockActor> nonOsgActor = std::make_shared<MockActor>("non-osg actor");
	EXPECT_FALSE(group->addActor(nonOsgActor)) << "OsgGroup should only succeed on actors that derive from OsgActor!";
	EXPECT_EQ(1u, group->getActors().size());
	EXPECT_EQ(group->getActors().end(), std::find(group->getActors().begin(), group->getActors().end(), nonOsgActor));
	EXPECT_EQ(2u, osgSwitch->getNumChildren());

	/// Try to add a non-OSG group
	std::shared_ptr<MockGroup> nonOsgSubgroup = std::make_shared<MockGroup>("non-osg group");
	EXPECT_FALSE(group->addGroup(nonOsgSubgroup)) <<
		"OsgGroup should only succeed on groups that derive from OsgGroup!";
	EXPECT_EQ(1u, group->getGroups().size());
	EXPECT_EQ(group->getGroups().end(), std::find(group->getGroups().begin(),group->getGroups().end(), nonOsgSubgroup));
	EXPECT_EQ(2u, osgSwitch->getNumChildren());
}

TEST(OsgGroupTests, ClearTests)
{
	std::shared_ptr<OsgGroup> osgGroup = std::make_shared<OsgGroup>("test name");
	std::shared_ptr<Group> group = osgGroup;

	std::shared_ptr<Actor> actor1 = std::make_shared<MockOsgActor>("test actor 1");
	std::shared_ptr<Actor> actor2 = std::make_shared<MockOsgActor>("test actor 2");
	std::shared_ptr<Group> group1 = std::make_shared<OsgGroup>("test group 1");
	std::shared_ptr<Group> group2 = std::make_shared<OsgGroup>("test group 2");

	osg::ref_ptr<osg::Switch> osgSwitch = dynamic_cast<osg::Switch*>(osgGroup->getOsgGroup().get());
	ASSERT_TRUE(osgSwitch.valid()) << "Group's OSG node should be a switch!";

	EXPECT_EQ(0u, group->getActors().size());
	EXPECT_EQ(0u, group->getGroups().size());
	EXPECT_EQ(0u, osgSwitch->getNumChildren());

	// Add actors and groups
	group->addActor(actor1);
	group->addActor(actor2);
	group->addGroup(group1);
	group->addGroup(group2);
	EXPECT_EQ(2u, group->getActors().size());
	EXPECT_EQ(2u, group->getGroups().size());
	EXPECT_EQ(4u, osgSwitch->getNumChildren());

	// Remove all actors
	group->clearActors();
	EXPECT_EQ(0u, group->getActors().size());
	EXPECT_EQ(2u, group->getGroups().size());
	EXPECT_EQ(2u, osgSwitch->getNumChildren());

	// Add actors again
	group->addActor(actor1);
	group->addActor(actor2);
	EXPECT_EQ(2u, group->getActors().size());
	EXPECT_EQ(2u, group->getGroups().size());
	EXPECT_EQ(4u, osgSwitch->getNumChildren());

	// Remove all groups
	group->clearGroups();
	EXPECT_EQ(2u, group->getActors().size());
	EXPECT_EQ(0u, group->getGroups().size());
	EXPECT_EQ(2u, osgSwitch->getNumChildren());

	// Add groups again
	group->addGroup(group1);
	group->addGroup(group2);
	EXPECT_EQ(2u, group->getActors().size());
	EXPECT_EQ(2u, group->getGroups().size());
	EXPECT_EQ(4u, osgSwitch->getNumChildren());

	// Remove everything
	group->clear();
	EXPECT_EQ(0u, group->getActors().size());
	EXPECT_EQ(0u, group->getGroups().size());
	EXPECT_EQ(0u, osgSwitch->getNumChildren());
}
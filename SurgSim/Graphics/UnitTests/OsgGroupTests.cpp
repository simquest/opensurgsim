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
#include "SurgSim/Graphics/UnitTests/MockOsgObjects.h"

#include "SurgSim/Graphics/OsgGroup.h"

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

	/// Add an representation and make sure the osg::Switch value for it is set correctly (should be true)
	std::shared_ptr<OsgRepresentation> representation1 =
		std::make_shared<MockOsgRepresentation>("test representation 1");
	group->add(representation1);
	EXPECT_EQ(1u, group->getMembers().size());
	EXPECT_EQ(1u, osgSwitch->getNumChildren());
	EXPECT_EQ(0u, osgSwitch->getChildIndex(representation1->getOsgNode()));
	EXPECT_TRUE(osgSwitch->getChildValue(representation1->getOsgNode())) << "Representation 1 should be visible!";

	/// Set group to not visible and check the osg::Switch values
	group->setVisible(false);
	EXPECT_FALSE(group->isVisible());
	EXPECT_FALSE(osgSwitch->getChildValue(representation1->getOsgNode())) << "Representation 1 should not be visible!";

	/// Add another representation and make sure the osg::Switch value for it is set correctly (should be false)
	std::shared_ptr<OsgRepresentation> representation2 =
		std::make_shared<MockOsgRepresentation>("test representation 2");
	group->add(representation2);
	EXPECT_EQ(2u, group->getMembers().size());
	EXPECT_EQ(2u, osgSwitch->getNumChildren());
	EXPECT_EQ(1u, osgSwitch->getChildIndex(representation2->getOsgNode()));
	EXPECT_FALSE(osgSwitch->getChildValue(representation2->getOsgNode())) << "Representation 2 should not be visible!";

	/// Set group to visible and check the osg::Switch values
	group->setVisible(true);
	EXPECT_TRUE(group->isVisible());
	EXPECT_TRUE(osgSwitch->getChildValue(representation1->getOsgNode())) << "Representation 1 should be visible!";
	EXPECT_TRUE(osgSwitch->getChildValue(representation2->getOsgNode())) << "Representation 2 should be visible!";

	/// Set group to not visible and check the osg::Switch values
	group->setVisible(false);
	EXPECT_FALSE(group->isVisible());
	EXPECT_FALSE(osgSwitch->getChildValue(representation1->getOsgNode())) << "Representation 1 should not be visible!";
	EXPECT_FALSE(osgSwitch->getChildValue(representation2->getOsgNode())) << "Representation 1 should not be visible!";

	/// Try to add a duplicate representation
	EXPECT_FALSE(group->add(representation1));
	EXPECT_EQ(2u, group->getMembers().size());
	EXPECT_EQ(2u, osgSwitch->getNumChildren());

	/// Remove an representation
	EXPECT_TRUE(group->remove(representation1));
	EXPECT_EQ(1u, osgSwitch->getNumChildren());
	EXPECT_EQ(1u, osgSwitch->getChildIndex(representation1->getOsgNode()));

	/// Try to remove an representation that is not in the group
	EXPECT_FALSE(group->remove(representation1));
	EXPECT_EQ(1u, osgSwitch->getChildIndex(representation1->getOsgNode()));

	/// Try to add a non-OSG representation
	std::shared_ptr<MockRepresentation> nonOsgRepresentation =
		std::make_shared<MockRepresentation>("non-osg representation");
	EXPECT_FALSE(group->add(nonOsgRepresentation)) <<
		"OsgGroup should only succeed on representations that derive from OsgRepresentation!";
	EXPECT_EQ(1u, group->getMembers().size());
	EXPECT_EQ(group->getMembers().end(),
		std::find(group->getMembers().begin(), group->getMembers().end(), nonOsgRepresentation)) <<
		"Only subclasses of OsgRepresentation should be in an OsgGroup!";
	EXPECT_EQ(1u, osgSwitch->getNumChildren());
}

TEST(OsgGroupTests, AppendTest)
{
	std::shared_ptr<Group> group1 = std::make_shared<OsgGroup>("test group 1");
	EXPECT_EQ(0u, group1->getMembers().size());

	std::shared_ptr<Representation> representation1 = std::make_shared<MockOsgRepresentation>("test representation 1");
	std::shared_ptr<Representation> representation2 = std::make_shared<MockOsgRepresentation>("test representation 2");

	/// Add 2 representations to group 1
	EXPECT_TRUE(group1->add(representation1));
	EXPECT_TRUE(group1->add(representation2));
	EXPECT_EQ(2u, group1->getMembers().size());

	std::shared_ptr<Representation> representation3 = std::make_shared<MockOsgRepresentation>("test representation 3");

	std::shared_ptr<Group> group2 = std::make_shared<OsgGroup>("test group 2");
	EXPECT_EQ(0u, group2->getMembers().size());

	/// Add an representation to group 2 and append group 1 to group 2.
	EXPECT_TRUE(group2->add(representation3));
	EXPECT_TRUE(group2->append(group1));
	EXPECT_EQ(3u, group2->getMembers().size());

	// Check that the representations from group 1 were added to group 2, and that it still has the representation
	// that was added directly to it.
	EXPECT_NE(group2->getMembers().end(),
		std::find(group2->getMembers().begin(), group2->getMembers().end(), representation1));
	EXPECT_NE(group2->getMembers().end(),
		std::find(group2->getMembers().begin(), group2->getMembers().end(), representation2));
	EXPECT_NE(group2->getMembers().end(),
		std::find(group2->getMembers().begin(), group2->getMembers().end(), representation3));

	/// Try to append a group that has already been appended - this will try to add duplicate representations.
	EXPECT_FALSE(group2->append(group1)) << "Append should return false if any representation is a duplicate!";
	EXPECT_EQ(3u, group2->getMembers().size());

	/// Check that group 1 was not modified by appending it to group 2.
	EXPECT_EQ(2u, group1->getMembers().size());
	EXPECT_NE(group1->getMembers().end(),
		std::find(group1->getMembers().begin(), group1->getMembers().end(), representation1));
	EXPECT_NE(group1->getMembers().end(),
		std::find(group1->getMembers().begin(), group1->getMembers().end(), representation2));

	/// Try to append a group that is not a subclass of OsgGroup
	std::shared_ptr<Group> nonOsgGroup = std::make_shared<MockGroup>("non-osg group");
	std::shared_ptr<Representation> nonOsgRepresentation =
		std::make_shared<MockRepresentation>("non-osg representation");
	/// Add an OSG and non-OSG representation to this group.
	EXPECT_TRUE(nonOsgGroup->add(representation3));
	EXPECT_TRUE(nonOsgGroup->add(nonOsgRepresentation));

	EXPECT_FALSE(group1->append(nonOsgGroup));
	EXPECT_EQ(2u, group1->getMembers().size()) <<
		"Nothing from the non-OSG group should have been added to the OsgGroup!";
	EXPECT_EQ(group1->getMembers().end(),
		std::find(group1->getMembers().begin(), group1->getMembers().end(), representation3));
	EXPECT_EQ(group1->getMembers().end(),
		std::find(group1->getMembers().begin(), group1->getMembers().end(), nonOsgRepresentation));
}

TEST(OsgGroupTests, ClearTest)
{
	std::shared_ptr<OsgGroup> osgGroup = std::make_shared<OsgGroup>("test name");
	std::shared_ptr<Group> group = osgGroup;

	std::shared_ptr<Representation> representation1 = std::make_shared<MockOsgRepresentation>("test representation 1");
	std::shared_ptr<Representation> representation2 = std::make_shared<MockOsgRepresentation>("test representation 2");
	std::shared_ptr<Representation> representation3 = std::make_shared<MockOsgRepresentation>("test representation 3");

	osg::ref_ptr<osg::Switch> osgSwitch = dynamic_cast<osg::Switch*>(osgGroup->getOsgGroup().get());
	ASSERT_TRUE(osgSwitch.valid()) << "Group's OSG node should be a switch!";

	EXPECT_EQ(0u, group->getMembers().size());
	EXPECT_EQ(0u, osgSwitch->getNumChildren());

	// Add 3 representations
	EXPECT_TRUE(group->add(representation1));
	EXPECT_TRUE(group->add(representation2));
	EXPECT_TRUE(group->add(representation3));
	EXPECT_EQ(3u, group->getMembers().size());
	EXPECT_EQ(3u, osgSwitch->getNumChildren());

	// Remove all representations
	group->clear();
	EXPECT_EQ(0u, group->getMembers().size());
	EXPECT_EQ(0u, osgSwitch->getNumChildren());
}

}  // namespace Graphics
}  // namespace SurgSim

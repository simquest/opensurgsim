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
/// Tests for the OsgManager class.

#include <SurgSim/Graphics/UnitTests/MockObjects.h>
#include <SurgSim/Graphics/UnitTests/MockOsgObjects.h>

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Graphics/OsgActor.h>
#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgView.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgViewElement.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <random>

using SurgSim::Framework::ComponentManager;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;

namespace SurgSim
{
namespace Graphics
{

TEST(OsgManagerTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();});
}

TEST(OsgManagerTests, StartUpTest)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);

	std::shared_ptr<Scene> scene = std::make_shared<Scene>();
	runtime->setScene(scene);

	/// Add a graphics component to the scene
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("test view element");
	scene->addSceneElement(viewElement);

	/// Run the thread for a moment
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	runtime->stop();
}

bool hasView(osgViewer::CompositeViewer* compositeViewer, osg::View* view)
{
	bool foundView = false;
	for (unsigned int i = 0; i < compositeViewer->getNumViews(); ++i)
	{
		if (compositeViewer->getView(i) == view)
		{
			foundView = true;
		}
	}
	return foundView;
}

TEST(OsgManagerTests, AddRemoveTest)
{
	std::shared_ptr<OsgManager> graphicsManager = std::make_shared<OsgManager>();
	osgViewer::CompositeViewer* compositeViewer = graphicsManager->getOsgCompositeViewer();
	/// Perform add and remove from a pointer to a ComponentManager to check that the intended polymorphism is working.
	std::shared_ptr<ComponentManager> componentManager = graphicsManager;

	std::shared_ptr<OsgActor> actor1 = std::make_shared<MockOsgActor>("test actor 1");
	std::shared_ptr<OsgActor> actor2 = std::make_shared<MockOsgActor>("test actor 2");
	std::shared_ptr<OsgGroup> group1 = std::make_shared<OsgGroup>("test group 1");
	std::shared_ptr<OsgGroup> group2 = std::make_shared<OsgGroup>("test group 2");
	std::shared_ptr<OsgView> view1 = std::make_shared<OsgView>("test view 1");
	std::shared_ptr<OsgView> view2 = std::make_shared<OsgView>("test view 2");
	std::shared_ptr<MockActor> nonOsgActor = std::make_shared<MockActor>("non-osg actor");
	std::shared_ptr<MockGroup> nonOsgGroup = std::make_shared<MockGroup>("non-osg group");
	std::shared_ptr<MockView> nonOsgView = std::make_shared<MockView>("non-osg view");
	using SurgSim::Framework::Representation;
	std::shared_ptr<Representation> nonGraphicsComponent = std::make_shared<Representation>("non-graphics component");

	EXPECT_EQ(0u, graphicsManager->getActors().size());
	EXPECT_EQ(0u, graphicsManager->getGroups().size());
	EXPECT_EQ(0u, graphicsManager->getViews().size());

	/// Add an actor
	EXPECT_TRUE(componentManager->addComponent(actor1));
	EXPECT_EQ(1u, graphicsManager->getActors().size());
	EXPECT_NE(graphicsManager->getActors().end(), std::find(graphicsManager->getActors().begin(),
		graphicsManager->getActors().end(), actor1));

	/// Add a group
	EXPECT_TRUE(componentManager->addComponent(group1));
	EXPECT_EQ(1u, graphicsManager->getGroups().size());
	EXPECT_NE(graphicsManager->getGroups().end(), std::find(graphicsManager->getGroups().begin(),
		graphicsManager->getGroups().end(), group1));

	/// Add a view
	EXPECT_TRUE(componentManager->addComponent(view1));
	EXPECT_EQ(1u, graphicsManager->getViews().size());
	EXPECT_NE(graphicsManager->getViews().end(), std::find(graphicsManager->getViews().begin(),
		graphicsManager->getViews().end(), view1));
	EXPECT_TRUE(hasView(compositeViewer, view1->getOsgView()));

	/// Add another view
	EXPECT_TRUE(componentManager->addComponent(view2));
	EXPECT_EQ(2u, graphicsManager->getViews().size());
	EXPECT_NE(graphicsManager->getViews().end(), std::find(graphicsManager->getViews().begin(),
		graphicsManager->getViews().end(), view2));

	/// Add another group
	EXPECT_TRUE(componentManager->addComponent(group2));
	EXPECT_EQ(2u, graphicsManager->getGroups().size());
	EXPECT_NE(graphicsManager->getGroups().end(), std::find(graphicsManager->getGroups().begin(),
		graphicsManager->getGroups().end(), group2));

	/// Add another actor
	EXPECT_TRUE(componentManager->addComponent(actor2));
	EXPECT_EQ(2u, graphicsManager->getActors().size());
	EXPECT_NE(graphicsManager->getActors().end(), std::find(graphicsManager->getActors().begin(),
		graphicsManager->getActors().end(), actor2));


	/// Try to add a duplicate actor
	EXPECT_FALSE(componentManager->addComponent(actor1));
	EXPECT_EQ(2u, graphicsManager->getActors().size());

	/// Try to add a duplicate group
	EXPECT_FALSE(componentManager->addComponent(group2));
	EXPECT_EQ(2u, graphicsManager->getGroups().size());

	/// Try to add a duplicate view
	EXPECT_FALSE(componentManager->addComponent(view1));
	EXPECT_EQ(2u, graphicsManager->getViews().size());

	/// Try to add an actor that is not a subclass of OsgActor
	EXPECT_FALSE(componentManager->addComponent(nonOsgActor)) <<
		"Adding an Actor that is not a subclass of OsgActor should fail and return false";
	EXPECT_EQ(2u, graphicsManager->getActors().size());

	/// Try to add a group that is not a subclass of OsgGroup
	EXPECT_FALSE(componentManager->addComponent(nonOsgGroup)) <<
		"Adding a Group that is not a subclass of OsgGroup should fail and return false";
	EXPECT_EQ(2u, graphicsManager->getGroups().size());

	/// Try to add a group that is not a subclass of OsgView
	EXPECT_FALSE(componentManager->addComponent(nonOsgView)) <<
		"Adding a View that is not a subclass of OsgView should fail and return false";
	EXPECT_EQ(2u, graphicsManager->getViews().size());

	/// Try to add a component that is not graphics-related
	EXPECT_TRUE(componentManager->addComponent(nonGraphicsComponent)) <<
		"Adding a component that this manager is not concerned with should return true";


	/// Remove a group
	EXPECT_TRUE(componentManager->removeComponent(group2));
	EXPECT_EQ(graphicsManager->getGroups().end(), std::find(graphicsManager->getGroups().begin(),
		graphicsManager->getGroups().end(), group2));

	/// Remove a view
	EXPECT_TRUE(componentManager->removeComponent(view2));
	EXPECT_EQ(graphicsManager->getViews().end(), std::find(graphicsManager->getViews().begin(),
		graphicsManager->getViews().end(), view2));

	/// Remove an actor
	EXPECT_TRUE(componentManager->removeComponent(actor1));
	EXPECT_EQ(graphicsManager->getActors().end(), std::find(graphicsManager->getActors().begin(),
		graphicsManager->getActors().end(), actor1));

	/// Try to remove a group that is not in the manager
	EXPECT_FALSE(componentManager->removeComponent(group2));
	EXPECT_EQ(graphicsManager->getGroups().end(), std::find(graphicsManager->getGroups().begin(),
		graphicsManager->getGroups().end(), group2));

	/// Try to remove an actor that is not in the manager
	EXPECT_FALSE(componentManager->removeComponent(actor1));
	EXPECT_EQ(graphicsManager->getActors().end(), std::find(graphicsManager->getActors().begin(),
		graphicsManager->getActors().end(), actor1));

	/// Try to remove a view that is not in the manager
	EXPECT_FALSE(componentManager->removeComponent(view2));
	EXPECT_EQ(graphicsManager->getViews().end(), std::find(graphicsManager->getViews().begin(),
		graphicsManager->getViews().end(), view2));


	/// Try to remove a component that is not graphics-related
	EXPECT_TRUE(componentManager->removeComponent(nonGraphicsComponent)) <<
		"Removing a component that this manager is not concerned with should return true";
}

}  // namespace Graphics
}  // namespace SurgSim

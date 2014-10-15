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
/// Tests for the Graphics Manager class.

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/ComponentManager.h"
#include "SurgSim/Graphics/UnitTests/MockObjects.h"

#include <gtest/gtest.h>

#include <algorithm>

using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::ComponentManager;
using SurgSim::Framework::Component;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;

class GraphicsManagerTest : public ::testing::Test
{
public:
	virtual void SetUp()
	{
		runtime = std::make_shared<Runtime>();
		graphicsManager = std::make_shared<MockManager>();

		runtime->addManager(graphicsManager);
		// runtime->start();
	}

	virtual void TearDown()
	{
		runtime->stop();
	}


	bool testDoAddComponent(const std::shared_ptr<Component>& component)
	{
		return graphicsManager->executeAdditions(component);
	}

	bool testDoRemoveComponent(const std::shared_ptr<Component>& component)
	{
		return graphicsManager->executeRemovals(component);
	}

	void doProcessComponents()
	{
		graphicsManager->processComponents();
	}

	std::shared_ptr<Runtime> runtime;
	std::shared_ptr<MockManager> graphicsManager;
};

namespace SurgSim
{

namespace Graphics
{

TEST_F(GraphicsManagerTest, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<MockManager> manager = std::make_shared<MockManager>();});
}

TEST_F(GraphicsManagerTest, StartUpTest)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<MockManager> manager = std::make_shared<MockManager>();

	runtime->addManager(manager);
	EXPECT_EQ(0, manager->getNumUpdates());
	EXPECT_EQ(0.0, manager->getSumDt());

	std::shared_ptr<Scene> scene = runtime->getScene();

	/// Run the thread for a moment
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	runtime->stop();

	/// Check that the manager did update when the thread was running
	EXPECT_GT(manager->getNumUpdates(), 0);
	EXPECT_GT(manager->getSumDt(), 0.0);
}

TEST_F(GraphicsManagerTest, AddRemoveTest)
{
	/// Perform add and remove from a pointer to a ComponentManager to check that the intended polymorphism is working.
	std::shared_ptr<ComponentManager> componentManager = graphicsManager;

	std::shared_ptr<Representation> representation1 = std::make_shared<MockRepresentation>("test representation 1");
	std::shared_ptr<Representation> representation2 = std::make_shared<MockRepresentation>("test representation 2");
	representation2->addGroupReference("other_group");
	std::shared_ptr<MockView> view1 = std::make_shared<MockView>("test view 1");
	std::shared_ptr<MockView> view2 = std::make_shared<MockView>("test view 2");

	std::shared_ptr<SurgSim::Framework::Representation> nonGraphicsComponent =
		std::make_shared<NonGraphicsRepresentation>("non-graphics component");

	EXPECT_EQ(0u, graphicsManager->getRepresentations().size());
	EXPECT_EQ(0u, graphicsManager->getGroups().size());
	EXPECT_EQ(0u, graphicsManager->getViews().size());

	/// Add an representation
	EXPECT_TRUE(testDoAddComponent(representation1));
	EXPECT_EQ(1u, graphicsManager->getRepresentations().size());
	EXPECT_NE(graphicsManager->getRepresentations().end(), std::find(graphicsManager->getRepresentations().begin(),
			  graphicsManager->getRepresentations().end(), representation1));

	// Should have added the default group
	EXPECT_EQ(1u, graphicsManager->getGroups().size());
	EXPECT_NE(std::end(graphicsManager->getGroups()),
			  graphicsManager->getGroups().find(Representation::DefaultGroupName));

	/// Add a view
	EXPECT_TRUE(testDoAddComponent(view1));
	EXPECT_EQ(1u, graphicsManager->getViews().size());
	EXPECT_NE(graphicsManager->getViews().end(), std::find(graphicsManager->getViews().begin(),
			  graphicsManager->getViews().end(), view1));


	/// Add another view
	EXPECT_TRUE(testDoAddComponent(view2));
	EXPECT_EQ(2u, graphicsManager->getViews().size());
	EXPECT_NE(graphicsManager->getViews().end(), std::find(graphicsManager->getViews().begin(),
			  graphicsManager->getViews().end(), view2));

	/// Add another representation
	EXPECT_TRUE(testDoAddComponent(representation2));
	EXPECT_EQ(2u, graphicsManager->getRepresentations().size());
	EXPECT_NE(graphicsManager->getRepresentations().end(), std::find(graphicsManager->getRepresentations().begin(),
			  graphicsManager->getRepresentations().end(), representation2));

	/// There should be another group in there now
	EXPECT_EQ(2u, graphicsManager->getGroups().size());

	/// Try to add a duplicate representation
	/// the public interface functions addComponent and removeComponent always return true when the allocation
	/// succeeded
	EXPECT_TRUE(componentManager->enqueueAddComponent(representation1));
	doProcessComponents();
	EXPECT_EQ(2u, graphicsManager->getRepresentations().size());

	/// Try to add a duplicate view
	EXPECT_TRUE(componentManager->enqueueAddComponent(view1));
	doProcessComponents();
	EXPECT_EQ(2u, graphicsManager->getViews().size());

	/// Try to add a component that is not graphics-related
	EXPECT_TRUE(componentManager->enqueueAddComponent(nonGraphicsComponent)) <<
			"Adding a component that this manager is not concerned with should return false";

	/// Remove a view
	EXPECT_TRUE(componentManager->enqueueRemoveComponent(view2));
	doProcessComponents();
	EXPECT_EQ(graphicsManager->getViews().end(), std::find(graphicsManager->getViews().begin(),
			  graphicsManager->getViews().end(), view2));

	/// Remove an representation
	EXPECT_TRUE(componentManager->enqueueRemoveComponent(representation1));
	doProcessComponents();
	EXPECT_EQ(graphicsManager->getRepresentations().end(), std::find(graphicsManager->getRepresentations().begin(),
			  graphicsManager->getRepresentations().end(), representation1));

	/// Try to remove an representation that is not in the manager
	EXPECT_TRUE(componentManager->enqueueRemoveComponent(representation1));
	doProcessComponents();
	EXPECT_EQ(graphicsManager->getRepresentations().end(), std::find(graphicsManager->getRepresentations().begin(),
			  graphicsManager->getRepresentations().end(), representation1));

	/// Try to remove a view that is not in the manager
	EXPECT_TRUE(componentManager->enqueueRemoveComponent(view2));
	doProcessComponents();
	EXPECT_EQ(graphicsManager->getViews().end(), std::find(graphicsManager->getViews().begin(),
			  graphicsManager->getViews().end(), view2));


	/// Try to remove a component that is not graphics-related
	EXPECT_TRUE(componentManager->enqueueRemoveComponent(nonGraphicsComponent)) <<
			"Removing a component that this manager is not concerned with should return true";
}

TEST_F(GraphicsManagerTest, UpdateTest)
{
	auto mockRepresentation = std::make_shared<MockRepresentation>("MockRepresentation");
	auto sceneElement = std::make_shared<BasicSceneElement>("BasicSceneElement");
	sceneElement->addComponent(mockRepresentation);
	runtime->getScene()->addSceneElement(sceneElement);

	EXPECT_TRUE(mockRepresentation->isActive());

	// When a graphics representation is inactive, it will be set to invisible by Graphics Manager.
	// And it won't be updated by Graphics Manager.
	mockRepresentation->setLocalActive(false);
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	EXPECT_EQ(0, mockRepresentation->getNumUpdates());
	EXPECT_FALSE(mockRepresentation->isActive());

	// When a graphics representation is active, it will be set to visible by Graphics Manager.
	// And it will be updated by Graphics Manager.
	mockRepresentation->setLocalActive(true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	EXPECT_GT(mockRepresentation->getNumUpdates(), 0);
	EXPECT_TRUE(mockRepresentation->isActive());

	// Turn off an active graphics representation, the update of it will be stopped and it will be invisible.
	mockRepresentation->setLocalActive(false);
	auto updateCount = mockRepresentation->getNumUpdates();
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	EXPECT_EQ(updateCount, mockRepresentation->getNumUpdates());
	EXPECT_FALSE(mockRepresentation->isActive());

	runtime->stop();
}

};  // namespace Graphics

};  // namespace SurgSim

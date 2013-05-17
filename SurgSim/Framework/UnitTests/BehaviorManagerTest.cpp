#include <gtest/gtest.h>

#include <SurgSim/Framework/BehaviorManager.h>

#include "MockObjects.h"
#include "boost/thread/thread.hpp"

using namespace SurgSim::Framework;

TEST(BehaviorManagerTest, AddRemoveTest)
{
	std::shared_ptr<BehaviorManager> manager(new BehaviorManager());
	std::shared_ptr<MockBehavior> behavior(new MockBehavior("Test Behavior1"));
	std::shared_ptr<MockBehavior> behavior2(new MockBehavior("Test Behavior2"));
	std::shared_ptr<MockComponent> component(new MockComponent("Test Component"));


	EXPECT_EQ(0,behavior->updateCount);
	EXPECT_TRUE(manager->addComponent(behavior));
	EXPECT_TRUE(manager->addComponent(behavior2));
	EXPECT_FALSE(manager->addComponent(behavior));

	// This should return true because the manager is not concerned 
	// with base components
	EXPECT_TRUE(manager->addComponent(component));

	EXPECT_TRUE(manager->removeComponent(behavior));
	EXPECT_FALSE(manager->removeComponent(behavior));
	EXPECT_TRUE(manager->removeComponent(behavior2));
	EXPECT_FALSE(manager->removeComponent(behavior2));
}

TEST(BehaviorManagerTest, BehaviorInitTest)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<BehaviorManager> behaviorManager(new BehaviorManager());

	runtime->addManager(behaviorManager);
	std::shared_ptr<Scene> scene(new Scene());
	std::shared_ptr<SceneElement> element(new MockSceneElement());
	std::shared_ptr<MockBehavior> behavior(new MockBehavior("MockBehavior"));

	element->addComponent(behavior);
	scene->addSceneElement(element);
	runtime->setScene(scene);

	runtime->start();
	EXPECT_TRUE(behaviorManager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	runtime->stop();

	EXPECT_TRUE(behavior->isInitialized);
	EXPECT_TRUE(behavior->isAwoken);
	EXPECT_TRUE(behavior->updateCount != 0);

}

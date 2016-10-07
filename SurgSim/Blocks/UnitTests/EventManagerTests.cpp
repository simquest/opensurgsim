// This file is a part of the OpenSurgSim project.
// Copyright 2013 - 2016, SimQuest Solutions Inc.
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
#include <gmock/gmock.h>

#include "SurgSim/Blocks/EventManager.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"

using ::testing::_;

namespace SurgSim
{
namespace Blocks
{

class MockReceiverInterface
{
public:
	virtual void onEventA(const EventManager::Event& event) = 0;
	virtual void onEventB(const EventManager::Event& event) = 0;
};

class MockReceiver : public SurgSim::Framework::Behavior, public MockReceiverInterface
{
public:
	explicit MockReceiver(const std::string& name) : SurgSim::Framework::Behavior(name) {}

	MOCK_METHOD1(onEventA, void(const EventManager::Event&));
	MOCK_METHOD1(onEventB, void(const EventManager::Event&));


	void update(double dt) override
	{
	}


	bool doInitialize() override
	{
		return true;
	}


	bool doWakeUp() override
	{
		return true;
	}

};

class EventManagerTest : public testing::Test
{
public:

	void SetUp()
	{
		runtime = std::make_shared<Framework::Runtime>();
		scene = runtime->getScene();
		sceneElement = std::make_shared<Framework::BasicSceneElement>("control");
		eventManager = std::make_shared<EventManager>("manager");
		sceneElement->addComponent(eventManager);
		sender = std::make_shared<MockReceiver>("sender");
		sceneElement->addComponent(sender);
		receiver1 = std::make_shared<MockReceiver>("receiver1");
		sceneElement->addComponent(receiver1);
		receiver2 = std::make_shared<MockReceiver>("receiver2");
		sceneElement->addComponent(receiver2);
		scene->addSceneElement(sceneElement);

	}

	std::shared_ptr<Framework::Runtime> runtime;
	std::shared_ptr<Framework::Scene> scene;
	std::shared_ptr<Framework::BasicSceneElement> sceneElement;
	std::shared_ptr<EventManager> eventManager;
	std::shared_ptr<MockReceiver> sender;
	std::shared_ptr<MockReceiver> receiver1;
	std::shared_ptr<MockReceiver> receiver2;
};

TEST_F(EventManagerTest, EmptyMessage)
{
	ASSERT_NO_THROW(eventManager->publish("sender", "event"));
	ASSERT_NO_THROW(eventManager->publish("sender", "event", 3));

	ASSERT_NO_THROW(eventManager->update(0.0));
}

TEST_F(EventManagerTest, SendMessage)
{
	using ::testing::Field;
	using ::testing::Eq;
	EXPECT_CALL(*receiver1, onEventA(AllOf(Field(&EventManager::Event::sender, Eq("sender")),
										   Field(&EventManager::Event::name, Eq("event"))))
			   ).Times(::testing::Exactly(1));
	auto callback = std::bind(&MockReceiver::onEventA, receiver1.get(), std::placeholders::_1);

	EXPECT_ANY_THROW(eventManager->subscribe("event", receiver1, nullptr));
	EXPECT_ANY_THROW(eventManager->subscribe("event", nullptr, callback));

	ASSERT_NO_THROW(eventManager->subscribe("event", receiver1, callback));


	ASSERT_NO_THROW(eventManager->publish("sender", "event"));
	ASSERT_NO_THROW(eventManager->update(0.0));

	// the queue should be empty, should not get another one
	ASSERT_NO_THROW(eventManager->update(0.0));
}

TEST_F(EventManagerTest, DoubleAdd)
{
	EXPECT_CALL(*receiver1, onEventA(_));

	auto callback = std::bind(&MockReceiver::onEventA, receiver1.get(), std::placeholders::_1);

	ASSERT_NO_THROW(eventManager->subscribe("event", receiver1, callback));
	ASSERT_NO_THROW(eventManager->subscribe("event", receiver1, callback));

	eventManager->publish("sender", "event");
	eventManager->update(0.0);
}


TEST_F(EventManagerTest, MultipleReceivers)
{
	EXPECT_CALL(*receiver1, onEventA(_));
	EXPECT_CALL(*receiver2, onEventB(_));

	auto callback1 = std::bind(&MockReceiver::onEventA, receiver1.get(), std::placeholders::_1);
	ASSERT_NO_THROW(eventManager->subscribe("event", receiver1, callback1));

	auto callback2 = std::bind(&MockReceiver::onEventB, receiver2.get(), std::placeholders::_1);
	ASSERT_NO_THROW(eventManager->subscribe("event", receiver2, callback2));

	ASSERT_NO_THROW(eventManager->publish("sender", "event"));
	ASSERT_NO_THROW(eventManager->update(0.0));
}

TEST_F(EventManagerTest, MultipleEvents)
{
	EXPECT_CALL(*receiver1, onEventA(_));
	EXPECT_CALL(*receiver2, onEventB(_));


	auto callback1 = std::bind(&MockReceiver::onEventA, receiver1.get(), std::placeholders::_1);
	ASSERT_NO_THROW(eventManager->subscribe("event1", receiver1, callback1));

	auto callback2 = std::bind(&MockReceiver::onEventB, receiver2.get(), std::placeholders::_1);
	ASSERT_NO_THROW(eventManager->subscribe("event2", receiver2, callback2));

	ASSERT_NO_THROW(eventManager->publish("sender", "event1"));
	ASSERT_NO_THROW(eventManager->publish("sender", "event2"));
	ASSERT_NO_THROW(eventManager->update(0.0));
}

TEST_F(EventManagerTest, Unsubscribe)
{
	EXPECT_CALL(*receiver1, onEventA(_));

	auto callback1 = std::bind(&MockReceiver::onEventA, receiver1.get(), std::placeholders::_1);
	eventManager->subscribe("event1", receiver1, callback1);

	auto callback2 = std::bind(&MockReceiver::onEventB, receiver1.get(), std::placeholders::_1);
	eventManager->subscribe("event2", receiver1, callback2);

	eventManager->publish("sender", "event1");
	eventManager->update(0.0);

	// remove from non-existing event
	ASSERT_NO_THROW(eventManager->unsubscribe("xxx", receiver2));

	// remove non-existing receiver
	ASSERT_NO_THROW(eventManager->unsubscribe("event1", receiver2));

	// actual removal
	ASSERT_NO_THROW(eventManager->unsubscribe("event2", receiver1));

	// double remove, should have no effect
	ASSERT_NO_THROW(eventManager->unsubscribe("event2", receiver1));

	// Now push a message and make sure we did not receive another one
	eventManager->publish("sender", "event2");
	eventManager->update(0.0);
}

TEST_F(EventManagerTest, Broadcast)
{
	EXPECT_CALL(*receiver1, onEventA(_)).Times(::testing::Exactly(2));

	auto callback = std::bind(&MockReceiver::onEventA, receiver1.get(), std::placeholders::_1);

	EXPECT_ANY_THROW(eventManager->subscribe(receiver1, nullptr));
	EXPECT_ANY_THROW(eventManager->subscribe(nullptr, callback));

	ASSERT_NO_THROW(eventManager->subscribe(receiver1, callback));

	eventManager->publish("sender", "event1");
	eventManager->publish("sender", "event2");
	eventManager->update(0.0);

	ASSERT_NO_THROW(eventManager->unsubscribe(receiver1));

	eventManager->publish("sender", "event1");
	eventManager->publish("sender", "event2");
	eventManager->update(0.0);

}

TEST_F(EventManagerTest, UnsubscribeFromAll)
{
	EXPECT_CALL(*receiver1, onEventA(_)).Times(::testing::Exactly(0));
	EXPECT_CALL(*receiver1, onEventB(_)).Times(::testing::Exactly(0));

	auto callback1 = std::bind(&MockReceiver::onEventA, receiver1.get(), std::placeholders::_1);
	eventManager->subscribe("event1", receiver1, callback1);

	auto callback2 = std::bind(&MockReceiver::onEventB, receiver1.get(), std::placeholders::_1);
	eventManager->subscribe("event2", receiver1, callback2);

	ASSERT_NO_THROW(eventManager->unsubscribe(receiver2));
	ASSERT_NO_THROW(eventManager->unsubscribe(receiver1));

	eventManager->publish("sender", "event1");
	eventManager->publish("sender", "event2");
	eventManager->update(0.0);
}

TEST_F(EventManagerTest, LifeTime)
{
	EXPECT_CALL(*receiver1, onEventA(_)).Times(::testing::Exactly(0));
	std::weak_ptr<SurgSim::Framework::Component> weak(receiver1);

	auto callback1 = std::bind(&MockReceiver::onEventA, receiver1.get(), std::placeholders::_1);
	eventManager->subscribe("event1", receiver1, callback1);

	sceneElement->removeComponent(receiver1);
	receiver1 = nullptr;

	// Verify that all the references are gone ...
	ASSERT_TRUE(weak.expired());

	eventManager->publish("sender", "event1");
	eventManager->publish("sender", "event2");
	ASSERT_NO_THROW(eventManager->update(0.0));
}

}
}
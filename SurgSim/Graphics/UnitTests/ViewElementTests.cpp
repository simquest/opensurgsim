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
/// Tests for the ViewElement class.

#include "SurgSim/Graphics/ViewElement.h"
#include "SurgSim/Graphics/UnitTests/MockObjects.h"

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"

#include <gtest/gtest.h>

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;

namespace SurgSim
{

namespace Graphics
{

/// Concrete ViewElement subclass for testing
class MockViewElement : public ViewElement
{
public:
	explicit MockViewElement(const std::string& name) : ViewElement(name, std::make_shared<MockView>(name + " View"))
	{
	}

	virtual std::shared_ptr<SurgSim::Input::CommonDevice> getKeyboardDevice() override
	{
		return nullptr;
	}

	virtual	void enableKeyboardDevice(bool val) override
	{
		return;
	}

	virtual std::shared_ptr<SurgSim::Input::CommonDevice> getMouseDevice() override
	{
		return nullptr;
	}

	virtual	void enableMouseDevice(bool val) override
	{
		return;
	}

};

TEST(ViewElementTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<ViewElement> element = std::make_shared<MockViewElement>("test name");});
}

TEST(ViewElementTests, StartUpTest)
{
	auto runtime = std::make_shared<Runtime>();
	auto manager = std::make_shared<MockManager>();

	runtime->addManager(manager);
	EXPECT_EQ(0, manager->getNumUpdates());
	EXPECT_EQ(0.0, manager->getSumDt());

	auto scene = std::make_shared<Scene>();
	runtime->setScene(scene);

	/// Add a graphics component to the scene
	auto viewElement = std::make_shared<MockViewElement>("Testing MockViewElement");
	scene->addSceneElement(viewElement);

	/// Run the thread for a moment
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	runtime->stop();

	/// Check that the view element was initialized and awoken
	EXPECT_TRUE(viewElement->isInitialized());
	EXPECT_TRUE(viewElement->isAwake());
}

TEST(ViewElementTests, ViewTest)
{
	std::shared_ptr<ViewElement> element = std::make_shared<MockViewElement>("Testing MockViewElement");

	/// Setting a MockView should succeed
	std::shared_ptr<View> mockView = std::make_shared<MockView>("Testing MockView");
	EXPECT_TRUE(element->setView(mockView));
	EXPECT_EQ(mockView, element->getView());
}

};  // namespace Graphics

};  // namespace SurgSim

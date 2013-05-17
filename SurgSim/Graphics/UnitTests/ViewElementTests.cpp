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

#include "SurgSim/Graphics/UnitTests/MockObjects.h"

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"

#include "gtest/gtest.h"

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Graphics::View;
using SurgSim::Graphics::ViewElement;

/// View class for testing adding a non-MockView
class NotMockView : public SurgSim::Graphics::View
{
public:
	/// Constructor
	/// \param	name	Name of the view
	explicit NotMockView(const std::string& name) : SurgSim::Graphics::View(name)
	{
	}

	/// Set the position of this view
	/// \param	x,y	Position on the screen (in pixels)
	virtual bool setPosition(int x, int y)
	{
		return true;
	}

	/// Get the position of this view
	/// \param[out]	x,y	Position on the screen (in pixels)
	virtual void getPosition(int* x, int* y)
	{
		*x = 0;
		*y = 0;
	}

	/// Set the dimensions of this view
	/// \param	width,height	Dimensions on the screen (in pixels)
	virtual bool setDimensions(int width, int height)
	{
		return true;
	}

	/// Set the dimensions of this view
	/// \param[out]	width,height	Dimensions on the screen (in pixels)
	virtual void getDimensions(int* width, int* height)
	{
		*width = 0;
		*height = 0;
	}

	/// Updates the view.
	virtual void update(double dt)
	{
	}

private:
	/// Initialize the view
	/// \post m_isInitialized is set to true
	virtual bool doInitialize()
	{
		return true;
	}
	/// Wake up the view
	/// \post m_isAwoken is set to true
	virtual bool doWakeUp()
	{
		return true;
	}
};

TEST(ViewElementTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<ViewElement> element = std::make_shared<MockViewElement>("test name");});
}

TEST(ViewElementTests, StartUpTest)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<MockManager> manager = std::make_shared<MockManager>();

	runtime->addManager(manager);
	EXPECT_EQ(0, manager->getNumUpdates());
	EXPECT_EQ(0.0, manager->getSumDt());

	std::shared_ptr<Scene> scene = std::make_shared<Scene>();
	runtime->setScene(scene);

	/// Add a graphics component to the scene
	std::shared_ptr<MockViewElement> viewElement = std::make_shared<MockViewElement>("test element");
	scene->addSceneElement(viewElement);

	/// Run the thread for a moment
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	runtime->stop();

	/// Check that the view element was initialized and awoken
	EXPECT_TRUE(viewElement->isInitialized());
	EXPECT_TRUE(viewElement->isAwoken());
	EXPECT_TRUE(viewElement->getMockView()->isInitialized());
	EXPECT_TRUE(viewElement->getMockView()->isAwoken());
}

TEST(ViewElementTests, ViewTest)
{
	std::shared_ptr<ViewElement> element = std::make_shared<MockViewElement>("test name");
	
	/// Setting a MockView should succeed
	std::shared_ptr<View> mockView = std::make_shared<MockView>("test mock view");
	EXPECT_TRUE(element->setView(mockView));
	EXPECT_EQ(mockView, element->getView());

	/// Any other View should fail
	std::shared_ptr<View> notMockView = std::make_shared<NotMockView>("test view, not a mock view");

	EXPECT_FALSE(element->setView(notMockView));
	EXPECT_NE(notMockView, element->getView());
	EXPECT_EQ(mockView, element->getView());
}

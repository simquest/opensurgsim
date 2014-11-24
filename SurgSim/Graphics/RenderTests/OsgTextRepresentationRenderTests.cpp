// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgTextRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/View.h"

#include <osg/Notify>

namespace SurgSim
{
namespace Graphics
{

struct OsgTextRepresentationRenderTests : public ::testing::Test
{
public:

	void SetUp()
	{
		runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
		graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();

		runtime->addManager(graphicsManager);
		scene = runtime->getScene();

		viewElement = std::make_shared<OsgViewElement>("view element");
		std::array<int, 2> position = {100, 100};
		viewElement->getView()->setPosition(position);
		viewElement->getView()->setWindowBorderEnabled(true);

		scene->addSceneElement(viewElement);
	}

	void TearDown()
	{
		runtime->stop();
	}

	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
	std::shared_ptr<OsgManager> graphicsManager;
	std::shared_ptr<SurgSim::Framework::Scene> scene;
	std::shared_ptr<OsgViewElement> viewElement;
};


TEST_F(OsgTextRepresentationRenderTests, InitTest)
{
	auto text =	std::make_shared<OsgTextRepresentation>("HUD");
	text->setText("HelloWorld");
	viewElement->addComponent(text);

	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

	auto dimensions = viewElement->getView()->getDimensions();
	text->setLocation(dimensions[0] / 2.0, dimensions[1] / 2.0);
	text->setText("Hello Again");
	boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

	text->setColor(SurgSim::Math::Vector4d(1.0, 0.5, 0.5, 1.0));
	text->setMaximumWidth(400.0);
	text->setFontSize(72.0);
	text->setText("This is a really long line that should be broken apart");
	boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
}

}; // namespace Graphics
}; // namespace SurgSim

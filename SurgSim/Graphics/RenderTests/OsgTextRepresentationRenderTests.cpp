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

#include "SurgSim/Blocks/PoseInterpolator.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgTextRepresentation.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/View.h"

#include "SurgSim/Graphics/RenderTests/RenderTest.h"

#include <osg/Notify>

namespace SurgSim
{
namespace Graphics
{

struct OsgTextRepresentationRenderTests : public SurgSim::Graphics::RenderTest
{

};


TEST_F(OsgTextRepresentationRenderTests, Operation)
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
	text->setDrawBackground(true);
	text->setBackgroundColor(Math::Vector4d(0.3, 0.3, 0.3, 1.0));
	boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
}

TEST_F(OsgTextRepresentationRenderTests, Background)
{
	{
		auto text = std::make_shared<OsgTextRepresentation>("Text1");
		text->setText("This is a sample text that should have a red box");
		text->setDrawBackground(true);
		viewElement->addComponent(text);
		text->setBackgroundColor(SurgSim::Math::Vector4d(1.0, 0.0, 0.0, 1.0));
		text->setColor(SurgSim::Math::Vector4d(1.0, 0.5, 0.5, 1.0));
		//text->setLocation(0, 100);
	}

	{
		auto text = std::make_shared<OsgTextRepresentation>("Text2");
		text->setText("This is a sample text that should have a blue box");
		text->setDrawBackground(true);
		viewElement->addComponent(text);
		text->setBackgroundColor(SurgSim::Math::Vector4d(0.0, 0.0, 1.0, 1.0));
		text->setColor(SurgSim::Math::Vector4d(1.0, 0.5, 0.5, 1.0));
		text->setLocation(100, 200);
	}

	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

}


TEST_F(OsgTextRepresentationRenderTests, WorldSpace)
{
	auto element = std::make_shared<Framework::BasicSceneElement>("Box");
	auto axis = std::make_shared<Graphics::OsgAxesRepresentation>("Axes");
	axis->setSize(0.1);
	element->addComponent(axis);
	auto box = std::make_shared<OsgBoxRepresentation>("Box");
	box->setSize(Math::Vector3d(0.05, 0.05, 0.05));
	element->addComponent(box);
	scene->addSceneElement(element);

	auto interpolator = std::make_shared<Blocks::PoseInterpolator>("Interpolator");
	interpolator->setTarget(viewElement);
	auto startingPose = Math::makeRigidTransform(
							Math::Vector3d(0.5, 0.5, 0.0),
							Math::Vector3d(0.0, 0.0, 0.0),
							Math::Vector3d(0.0, 1.0, 0.0));
	interpolator->setStartingPose(startingPose);
	interpolator->setEndingPose(Math::makeRigidTransform(
									Math::Vector3d(-0.5, 0.5, 0.0),
									Math::Vector3d(0.0, 0.0, 0.0),
									Math::Vector3d(0.0, 1.0, 0.0)));
	interpolator->setPingPong(true);
	interpolator->setDuration(6.0);
	viewElement->addComponent(interpolator);

	auto text = std::make_shared<OsgTextRepresentation>("Text");
	text->setText("This is a multiline text, it should be well readable and right in front of the camera");
	text->setUseScreenSpace(false);
	text->setMaximumWidth(0.4);
	text->setFontSize(0.02);
	text->setLocalPose(Math::makeRigidTranslation(Math::Vector3d(0.0, 0.0, -1.0)));
	viewElement->addComponent(text);

	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

	runtime->stop();
}

}; // namespace Graphics
}; // namespace SurgSim

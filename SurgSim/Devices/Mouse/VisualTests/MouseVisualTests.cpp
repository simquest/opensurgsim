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

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <osg/Camera>
#include <osg/Geode>
#include <osgText/Text>
#include <osgViewer/Viewer>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Mouse/MouseDevice.h"
#include "SurgSim/Devices/Mouse/OsgMouseHandler.h"
#include "SurgSim/Input/InputConsumerInterface.h"



using SurgSim::DataStructures::DataGroup;

struct TestListener : public SurgSim::Input::InputConsumerInterface
{
	void initializeInput(const std::string& device, const DataGroup& inputData) override
	{
	}

	void handleInput(const std::string& device, const DataGroup& inputData) override
	{
		bool button1, button2, button3;
		double x, y;
		int scrollDeltaX, scrollDeltaY;

		if (inputData.booleans().get(SurgSim::DataStructures::Names::BUTTON_1, &button1))
		{
			std::cerr << "button1 = " << button1 << std::endl;
		}
		if (inputData.booleans().get(SurgSim::DataStructures::Names::BUTTON_2, &button2))
		{
			std::cerr << "button2 = " << button2 << std::endl;
		}
		if (inputData.booleans().get(SurgSim::DataStructures::Names::BUTTON_3, &button3))
		{
			std::cerr << "button3 = " << button3 << std::endl;
		}
		if (inputData.scalars().get("x", &x))
		{
			std::cerr << "x = " << x << std::endl;
		}
		if (inputData.scalars().get("y", &y))
		{
			std::cerr << "y = " << y << std::endl;
		}
		if (inputData.integers().get("scrollDeltaX", &scrollDeltaX))
		{
			std::cerr << "scrollDeltaX = " << scrollDeltaX << std::endl;
		}
		if (inputData.integers().get("scrollDeltaY", &scrollDeltaY))
		{
			std::cerr << "scrollDeltaY = " << scrollDeltaY << std::endl;
		}

		std::cerr << std::endl;
	}
};

int main(int argc, char* argv[])
{
	auto toolDevice	 = std::make_shared<SurgSim::Device::MouseDevice>("Mouse");
	toolDevice->initialize();

	osg::ref_ptr<osgGA::GUIEventHandler> mouseHandler = toolDevice->getMouseHandler();
	auto consumer = std::make_shared<TestListener>();
	toolDevice->addInputConsumer(consumer);

	osg::ref_ptr<osgText::Text> text = new osgText::Text;
	text->setText("Move/click/drag mouse in\n\nthis window to verify that\n\nmouse driver works correctly.");
	text->setPosition(osg::Vec3(0.0f, 300.0f, 0.0f));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(text);

	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setProjectionMatrixAsOrtho2D(0, 600, 0, 400);
	camera->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	camera->addChild(geode);

	osg::ref_ptr<osg::Group> group = new osg::Group;
	group->addChild(camera);

	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
	viewer->setUpViewInWindow(400, 400, 640, 480);
	viewer->addEventHandler(mouseHandler);
	viewer->setSceneData(group);

	viewer->run();
	return 0;
}

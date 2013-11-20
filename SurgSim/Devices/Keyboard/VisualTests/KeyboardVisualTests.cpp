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

#include <SurgSim/Devices/Keyboard/KeyboardDevice.h>
#include <SurgSim/Devices/Keyboard/KeyboardHandler.h>

#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/Projection>
#include <osgGA/GUIEventHandler>
#include <osgText/Text>
#include <osgViewer/config/SingleWindow>
#include <osgViewer/Viewer>

int main(int argc, char* argv[])
{
	auto toolDevice	 = std::make_shared<SurgSim::Device::KeyboardDevice>("Keyboard");
	toolDevice->initialize();
	osg::ref_ptr<osgGA::GUIEventHandler> keyboardHandler = toolDevice->getKeyboardHandler();

	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
	viewer->apply(new osgViewer::SingleWindow(100, 200, 600, 400));
	viewer->addEventHandler(keyboardHandler);

	osg::ref_ptr<osg::Group> group = new osg::Group;
	viewer->setSceneData(group);

	osg::ref_ptr<osg::MatrixTransform> matrixTransform = new osg::MatrixTransform;
	matrixTransform->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

	osg::ref_ptr<osg::Projection> projection = new osg::Projection;
	projection->setMatrix(osg::Matrix::ortho2D(0, 600, -300, 400));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osgText::Text> text = new osgText::Text;
	text->setText("Press any key\nAnd the key code will\nbe shown in std::cerr");

	geode->addDrawable(text);
	projection->addChild(geode);
	matrixTransform->addChild(projection);
	group->addChild(matrixTransform);

	viewer->run();
	return 0;
}

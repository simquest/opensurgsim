//// This file is a part of the OpenSurgSim project.
//// Copyright 2013, SimQuest Solutions Inc.
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/InputConsumerInterface.h"

#include "SurgSim/Devices/TrackIR/TrackIRDevice.h"
#include "SurgSim/Devices/TrackIR/TrackIRScaffold.h"
#include "SurgSim/Math/RigidTransform.h"

#include <osg/Camera>
#include <osg/Geode>
#include <osgText/Text>
#include <osgViewer/Viewer>

using SurgSim::DataStructures::DataGroup;
using SurgSim::Device::TrackIRDevice;
using SurgSim::Math::RigidTransform3d;


struct TestListener : public SurgSim::Input::InputConsumerInterface
{
	virtual void initializeInput(const std::string& device, const DataGroup& inputData) override
	{
	}

	virtual void handleInput(const std::string& device, const DataGroup& inputData) override
	{
		RigidTransform3d pose;
		inputData.poses().get("pose", &pose);

	}
};

int main(int argc, char* argv[])
{
	auto toolDevice	 = std::make_shared<SurgSim::Device::TrackIRDevice>("TrackIR");
	toolDevice->initialize();

	auto consumer = std::make_shared<TestListener>();
	toolDevice->addInputConsumer(consumer);

	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setProjectionMatrixAsOrtho2D(0, 600, 0, 400);
	camera->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

	osg::ref_ptr<osg::Group> group = new osg::Group;
	group->addChild(camera);

	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
	viewer->setUpViewInWindow(400, 400, 640, 480);
	viewer->setSceneData(group);

	viewer->run();
	return 0;
}

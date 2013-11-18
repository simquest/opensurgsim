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

#include <memory>

#include <SurgSim/Devices/Keyboard/KeyboardDevice.h>

#include <SurgSim/Framework/BehaviorManager.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>

#include <SurgSim/Graphics/OsgBoxRepresentation.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Graphics/OsgView.h>

#include <SurgSim/Input/InputManager.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;


std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	std::shared_ptr<SurgSim::Graphics::ViewElement> viewElement = std::make_shared<SurgSim::Graphics::OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);
	viewElement->getView();

	auto inputComponent = std::make_shared<SurgSim::Input::InputComponent>("input", "Keyboard");
	/*std::shared_ptr<VirtualToolCoupler> inputCoupler =
	std::make_shared<VirtualToolCoupler>("Input Coupler", inputComponent, physicsRepresentation);*/
	viewElement->addComponent(inputComponent);
	//viewElement->addComponent(inputCoupler);



	//std::dynamic_pointer_cast<osgViewer::View>(viewElement->getView())->addEventHandler();

	return viewElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createBox(const std::string& name, double width, double height, double length) 
{
	std::shared_ptr<SurgSim::Framework::SceneElement> sceneElement 
		= std::make_shared<SurgSim::Graphics::OsgViewElement>(name);

	std::shared_ptr<SurgSim::Graphics::BoxRepresentation> boxRepresentation = 
		std::make_shared<SurgSim::Graphics::OsgBoxRepresentation>(name + "Graphics");
	boxRepresentation->setInitialPose(RigidTransform3d::Identity());
	boxRepresentation->setSize(width, height, length);
	sceneElement->addComponent(boxRepresentation);

	return sceneElement;
}

int main(int argc, char* argv[])
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	auto scene = std::make_shared<SurgSim::Framework::Scene>();
	auto behaviorManager = std::make_shared<SurgSim::Framework::BehaviorManager>();
	auto inputManager = std::make_shared<SurgSim::Input::InputManager>();

	auto toolDevice = std::make_shared<SurgSim::Device::KeyboardDevice>("Keyboard");
	SURGSIM_ASSERT( toolDevice->initialize() == true ) <<
		"Could not initialize device '%s' for the tool.\n", toolDevice->getName().c_str();
	inputManager->addDevice(toolDevice);

	osgGA::GUIEventHandler keyboardHandler = *(toolDevice->getKeyboardHandler().get());

	auto view = createView("View", 0, 0, 1023, 768);
	std::dynamic_pointer_cast<osgViewer::View>(view->getView())->addEventHandler(keyboardHandler.get());
	
	scene->addSceneElement(createBox("box", 1.0, 2.0, 3.0));
	scene->addSceneElement(view);

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 10.0)));

	runtime->addManager(graphicsManager);
	runtime->addManager(behaviorManager);
	runtime->addManager(inputManager);

	runtime->setScene(scene);
	runtime->execute();

	return 0;
}

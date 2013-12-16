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

#include "SurgSim/Blocks/BasicSceneElement.h"
#include "SurgSim/Blocks/TransferPoseBehavior.h"
#include "SurgSim/Blocks/TransferInputPoseBehavior.h"
#include "SurgSim/Devices/TrackIR/TrackIRDevice.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"


using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Blocks::TransferPoseBehavior;
using SurgSim::Blocks::TransferInputPoseBehavior;
using SurgSim::Framework::Logger;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Math::Vector3d;



std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);
	return viewElement;
}

std::shared_ptr<SceneElement> createBox(const std::string& name)
{
	auto graphicsRepresentation = std::make_shared<OsgBoxRepresentation>(name + "-Graphics");
	graphicsRepresentation->setSize(5.0, 4.0, 3.0);  // Size in meter
	auto inputComponent = std::make_shared<SurgSim::Input::InputComponent>("input", "TrackIRDevice");

	std::shared_ptr<SceneElement> boxElement = std::make_shared<BasicSceneElement>(name);
	boxElement->addComponent(graphicsRepresentation);
	boxElement->addComponent(inputComponent);
	boxElement->addComponent(std::make_shared<TransferInputPoseBehavior>("Input to Graphics",
								inputComponent, graphicsRepresentation));
	return boxElement;
}


int main(int argc, char* argv[])
{
	auto graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	auto behaviorManager = std::make_shared<SurgSim::Framework::BehaviorManager>();
	auto inputManager    = std::make_shared<SurgSim::Input::InputManager>();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::shared_ptr<SurgSim::Device::TrackIRDevice> toolDevice =
		std::make_shared<SurgSim::Device::TrackIRDevice>("TrackIRDevice");
	SURGSIM_ASSERT( toolDevice->initialize() == true ) <<
		"Could not initialize device '%s' for the tool.\n", toolDevice->getName().c_str();

	inputManager->addDevice(toolDevice);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::shared_ptr<SurgSim::Framework::Runtime> runtime(new SurgSim::Framework::Runtime());
	runtime->addManager(graphicsManager);
	runtime->addManager(behaviorManager);
	runtime->addManager(inputManager);

	std::shared_ptr<SurgSim::Framework::Scene> scene(new SurgSim::Framework::Scene());
	scene->addSceneElement(createBox("box"));
	scene->addSceneElement(createView("view", 0, 0, 640, 480));

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 100.0)));

	runtime->setScene(scene);
	runtime->execute();

	return 0;
}

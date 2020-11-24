// This file is a part of the OpenSurgSim project.
// Copyright 2020, SimQuest Solutions Inc.
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

#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Input/Input.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Physics/Physics.h"

#include "Library/DrawFunctions.h"
#include "Library/VisualDebugBehavior.h"
#include "Library/GlewInitOperation.h"

#include "imgui.h"
#include "backends/imgui_impl_opengl3.h"

using namespace SurgSim;


std::shared_ptr<Framework::Runtime> createRuntime(std::unordered_map<std::string, std::string> parameters =
			std::unordered_map<std::string, std::string>())
{

	Framework::Logger::getLoggerManager()->setThreshold(Framework::LOG_LEVEL_INFO);
	auto logger = Framework::Logger::getLogger("Editor");

	auto behaviorManager = std::make_shared<Framework::BehaviorManager>();
	auto graphicsManager = std::make_shared<Graphics::OsgManager>();
	auto inputManager = std::make_shared<Input::InputManager>();
	auto physicsManager = std::make_shared<Physics::PhysicsManager>();

	graphicsManager->setRate(60.0);

	auto runtime = std::make_shared<Framework::Runtime>("config.txt");
	runtime->addManager(behaviorManager);
	runtime->addManager(inputManager);
	runtime->addManager(graphicsManager);

	graphicsManager->getOsgCompositeViewer()->setRealizeOperation(new SurgSim::EditDebug::GlewInitOperation);

	runtime->addManager(physicsManager);

	auto scene = runtime->getScene();

 	auto monitorDisplay = std::make_shared<Graphics::OsgViewElement>("MonitorDisplay");
 	monitorDisplay->enableManipulator(true);
	monitorDisplay->getCamera()->setAmbientColor(Math::Vector4d(0.1, 0.1, 0.1, 1.0));
	inputManager->addDevice(monitorDisplay->getKeyboardDevice());
	scene->addSceneElement(monitorDisplay);


	{
		auto element = std::make_shared<Framework::BasicSceneElement>("Element");
		element->setPose(SurgSim::Math::makeRigidTranslation(SurgSim::Math::Vector3d(1.0, 2.0, 3.0)));
		auto axis = std::make_shared<Graphics::OsgAxesRepresentation>("Axes");
		element->addComponent(axis);
		scene->addSceneElement(element);
	}

	{
		auto element = std::make_shared<Framework::BasicSceneElement>("DebugElement");
		auto debugger = std::make_shared<SurgSim::EditDebug::VisualDebugBehavior>("Debugger");
		element->addComponent(debugger);
		scene->addSceneElement(element);
	}
	
	return runtime;
}

int main(int argc, char** argv)
{

	Framework::ApplicationData appData("config.txt");

	auto runtime = createRuntime();

	runtime->execute();

	return 0;
}

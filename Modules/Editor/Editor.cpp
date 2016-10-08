// Copyright 2016, SimQuest Solutions Inc.


#include "SurgSim/Blocks/Blocks.h"
#include "SurgSim/Devices/Devices.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Input/Input.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Physics/Physics.h"

#include "EditorManager.h"
#include "ImGuiHandler.h"
#include "imgui.h"
using namespace SurgSim;

struct Callback : public sf::GuiCallback
{
	void operator()()
	{
		ImGui::Text("This is a test");
	}
} callback;
std::shared_ptr<Framework::Runtime> createRuntime(std::unordered_map<std::string, std::string> parameters =
			std::unordered_map<std::string, std::string>())
{
	Framework::Logger::getLoggerManager()->setThreshold(Framework::LOG_LEVEL_INFO);
	auto logger = Framework::Logger::getLogger("Editor");

	auto behaviorManager = std::make_shared<Framework::BehaviorManager>();
	auto graphicsManager = std::make_shared<Graphics::EditorManager>();
	auto inputManager = std::make_shared<Input::InputManager>();
	auto physicsManager = std::make_shared<Physics::PhysicsManager>();

	graphicsManager->setRate(60.0);

	auto runtime = std::make_shared<Framework::Runtime>("config.txt");
	runtime->addManager(behaviorManager);
	runtime->addManager(inputManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(physicsManager);

	auto scene = runtime->getScene();

	auto monitorDisplay = std::make_shared<Graphics::OsgViewElement>("MonitorDisplay");
	monitorDisplay->enableManipulator(true);
	monitorDisplay->getCamera()->setAmbientColor(Math::Vector4d(0.1, 0.1, 0.1, 1.0));
	inputManager->addDevice(monitorDisplay->getKeyboardDevice());
	scene->addSceneElement(monitorDisplay);

	auto view = std::dynamic_pointer_cast<Graphics::OsgView>(monitorDisplay->getView());

	auto imGuiHandler = new sf::ImGuiHandler(&callback);
	view->getOsgView()->addEventHandler(imGuiHandler);

	auto camera = std::dynamic_pointer_cast<Graphics::OsgCamera>(monitorDisplay->getCamera());
	imGuiHandler->setCameraCallbacks(camera->getOsgCamera());

	auto element = std::make_shared<Framework::BasicSceneElement>("Element");
	auto axis = std::make_shared<Graphics::OsgAxesRepresentation>("Axes");
	element->addComponent(axis);
	scene->addSceneElement(element);

	return runtime;
}

int main(int argc, char** argv)
{
	Framework::ApplicationData appData("config.txt");

	auto runtime = createRuntime();

	runtime->execute();

	return 0;
}

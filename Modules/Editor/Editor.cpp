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


class ImGuiLogOutput : public Framework::LogOutput
{

public:
	virtual bool writeMessage(const std::string& message) override
	{
		boost::lock_guard<boost::mutex> lock(m_bufferMutex);
		m_messageBuffer.push_back(message);
		if (m_messageBuffer.size() > 20)
		{
			m_messageBuffer.pop_front();
		}
		return true;
	}

	void draw()
	{
		boost::lock_guard<boost::mutex> lock(m_bufferMutex);
		static bool open = true;
		if (ImGui::Begin("Log", &open))
		{
			for (const auto& line : m_messageBuffer)
			{
				ImGui::Text(line.c_str());
			}
		}
		ImGui::End();
	}

private:
	std::list<std::string> m_messageBuffer;
	boost::mutex m_bufferMutex;

};

struct Callback : public sf::GuiCallback
{
	Callback()
	{
		logger = std::make_shared<ImGuiLogOutput>();
		Framework::Logger::getLoggerManager()->setDefaultOutput(logger);

	}

	void operator()()
	{
		bool open = true;
		logger->draw();

		auto elements = scene->getSceneElements();
		for (const auto& element : elements)
		{
			if (ImGui::CollapsingHeader(element->getName().c_str()))
			{
				auto components = element->getComponents();
				for (const auto& component : components)
				{
					ImGui::PushID(component->getFullName().c_str());
					auto properties = component->getProperties();
					for (const auto& prop : properties)
					{
						inputProperty(prop, component.get());
					}

					ImGui::PopID();
				}
			}
		}

	}

	bool inputProperty(const std::string& property, Framework::Component* component)
	{
		auto boostValue = component->getValue(property);
		bool handled = false;
		bool edited = false;
		try
		{
			auto value = boost::any_cast<Math::RigidTransform3d>(boostValue);
			edited = InputRigidTransform(property, &value);
			if (edited)
			{
				component->setValue(property, value);
			}
			handled = true;
		}
		catch (std::exception e)
		{

		}
		try
		{
			auto value = boost::any_cast<double>(boostValue);
			auto floatValue = static_cast<float>(value);
			edited = ImGui::InputFloat(property.c_str(), &floatValue);
			if (edited)
			{
				component->setValue(property, static_cast<double>(floatValue));
			}
			handled = true;
		}
		catch (std::exception e)
		{

		}

		if (!handled)
		{
			auto text = property + "!!!";
			ImGui::Text(text.c_str());
		}

		return edited;
	}

	bool InputRigidTransform(const std::string title, Math::RigidTransform3d* pose)
	{
		ImGui::Text(title.c_str());
		auto translation = pose->translation().cast<float>().eval();
		bool changed;
		changed = ImGui::InputFloat("x", &translation[0]);
		changed = ImGui::InputFloat("y", &translation[1]) || changed;
		changed = ImGui::InputFloat("z", &translation[2]) || changed;
		if (changed == true)
		{
			pose->translation() = translation.cast<double>();
		}
		return changed;
	}


	std::shared_ptr<Framework::Scene> scene;
	std::shared_ptr<ImGuiLogOutput> logger;
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
	callback.scene = scene;

	return runtime;
}

int main(int argc, char** argv)
{
	Framework::ApplicationData appData("config.txt");

	auto runtime = createRuntime();

	runtime->execute();

	return 0;
}

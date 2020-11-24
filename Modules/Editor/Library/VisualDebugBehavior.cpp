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

#include "VisualDebugBehavior.h"

#include "GL/glew.h"

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/LogOutput.h"

#include "SurgSim/Graphics/View.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Graphics/OsgCamera.h"

#include "imgui.h"

#include "OsgImGuiHandler.h"
#include "DrawFunctions.h"

namespace
{
	template <class T> 
	std::vector<std::shared_ptr<T>> findObjects(const SurgSim::Framework::Scene* scene)
	{
		std::vector <std::shared_ptr<T>> result;
		for (const auto& element : scene->getSceneElements())
		{
			for (const auto& component : element->getComponents())
			{
				auto typed = std::dynamic_pointer_cast<T>(component);
				if (typed != nullptr)  result.emplace_back(std::move(typed));
			}
		}
		return result;
	}

	// temporary ... figure out architecture for multiple outputs 

	class ImGuiLogOutput : public SurgSim::Framework::LogOutput
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
		// 			logger = std::make_shared<ImGuiLogOutput>();
		// 			Framework::Logger::getLoggerManager()->setDefaultOutput(logger);

	};


}

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::EditDebug::VisualDebugBehavior, SurgSim::EditDebug::VisualDebugBehavior);


namespace SurgSim
{ 
namespace EditDebug
{
// Osg does reference counting, we could handle this inside the behavior 
// so it is safer to do use a separate class
class DebugGui : public OsgImGuiHandler
{
public:
	DebugGui(std::shared_ptr<SurgSim::EditDebug::VisualDebugBehavior> behavior) :
		m_behavior(behavior) {};

	void drawUi() override
	{
		auto dimensions = m_behavior->m_view->getDimensions();
		SurgSim::EditDebug::showSceneEditor(m_behavior->getRuntime().get(), dimensions[0], dimensions[1]);
		//static bool demoOpen = true; // Use this for reference when looking for solutions
		//ImGui::ShowDemoWindow(&demoOpen);
	}

	std::shared_ptr<SurgSim::EditDebug::VisualDebugBehavior> m_behavior;
};

VisualDebugBehavior::VisualDebugBehavior(const std::string& name) : Behavior(name)
{

}

bool VisualDebugBehavior::doInitialize()
{
	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	return true;
}



bool VisualDebugBehavior::doWakeUp()
{
	// Assumes only one view and one camera ... 
	// if not we will need to find the main view and the main camera 

	auto scene = getRuntime()->getScene();

	if (m_view == nullptr)
	{
		auto objects = findObjects<SurgSim::Graphics::View>(scene.get());
		SURGSIM_ASSERT(objects.size() > 0) << "Could not find a View in the Scene";
		m_view = objects[0];
	}

	if (m_camera == nullptr)
	{
		auto objects = findObjects<SurgSim::Graphics::Camera>(scene.get());
		SURGSIM_ASSERT(objects.size() > 0) << "Could not find a Camera in the Scene";
		m_camera = objects[0];
	}

	auto view = std::dynamic_pointer_cast<SurgSim::Graphics::OsgView>(m_view);
	SURGSIM_ASSERT(view != nullptr);

	auto guiHandler = new DebugGui(std::dynamic_pointer_cast<VisualDebugBehavior>(this->getSharedPtr()));

	view->getOsgView()->addEventHandler(guiHandler);
	auto camera = std::dynamic_pointer_cast<Graphics::OsgCamera>(m_camera);

	return true;
}

int VisualDebugBehavior::getTargetManagerType() const
{
	return SurgSim::Framework::MANAGER_TYPE_GRAPHICS;
}

void VisualDebugBehavior::update(double dt)
{

}

}
}


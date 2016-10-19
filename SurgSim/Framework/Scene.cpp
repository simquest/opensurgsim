// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest LLC.
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

#include "SurgSim/Framework/Scene.h"

#include <boost/thread/locks.hpp>
#include <set>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/SceneElement.h"


namespace SurgSim
{
namespace Framework
{

Scene::Scene(std::weak_ptr<Runtime> runtime) :
	m_runtime(runtime),
	m_logger(Framework::Logger::getLogger("Framework/Scene"))
{
	SURGSIM_ASSERT(!m_runtime.expired()) << "Can't create scene with empty runtime.";
}

Scene::~Scene()
{

}

void Scene::addSceneElement(std::shared_ptr<SceneElement> element)
{
	auto runtime = getRuntime();
	SURGSIM_ASSERT(runtime) << "Runtime pointer is expired, cannot add SceneElement to Scene.";

	std::string name = element->getName();
	element->setScene(getSharedPtr());
	element->setRuntime(runtime);

	if (element->initialize())
	{
		{
			boost::lock_guard<boost::mutex> lock(m_sceneElementsMutex);
			m_elements.push_back(element);
			m_groups.add(element->getGroups(), element);
		}
		runtime->addSceneElement(element);
	}
}

void Scene::removeSceneElement(std::shared_ptr<SceneElement> element)
{
	element->setActive(false);
	element->removeComponents();

	boost::lock_guard<boost::mutex> lock(m_sceneElementsMutex);
	auto found = std::find(m_elements.begin(),  m_elements.end(), element);
	if (found != m_elements.end())
	{
		m_elements.erase(found);
	}
	else
	{
		SURGSIM_LOG_WARNING(m_logger)
			<< "Could not find element '" << element->getName() << "' in Scene, unable to remove";
	}
}

void Scene::addSceneElements(std::vector<std::shared_ptr<SceneElement>> elements)
{
	for (auto element : elements)
	{
		addSceneElement(element);
	}
}

std::shared_ptr<Runtime> Scene::getRuntime()
{
	return m_runtime.lock();
}

const std::vector <std::shared_ptr<SceneElement>>& Scene::getSceneElements() const
{
	return m_elements;
}

const std::shared_ptr<SceneElement> Scene::getSceneElement(const std::string& name) const
{
	std::shared_ptr<SceneElement> result = nullptr;
	for (auto it = std::begin(m_elements); it != std::end(m_elements); ++it)
	{
		if ((*it)->getName() == name)
		{
			result = *it;
			break;
		}
	}

	return result;
}

std::shared_ptr<Scene> Scene::getSharedPtr()
{
	std::shared_ptr<Scene> result;
	try
	{
		result = shared_from_this();
	}
	catch (const std::exception&)
	{
		SURGSIM_FAILURE() << "Scene was not created as a shared_ptr";
	}
	return result;
}

YAML::Node Scene::encode() const
{
	YAML::Node result(YAML::NodeType::Map);
	YAML::Node data(YAML::NodeType::Map);
	for (auto sceneElement = m_elements.begin(); sceneElement != m_elements.end(); ++sceneElement)
	{
		data["SceneElements"].push_back(*(*sceneElement));
	}

	result["SurgSim::Framework::Scene"] = data;

	return result;
}

bool Scene::decode(const YAML::Node& node)
{
	bool result = false;
	if (node.IsMap())
	{
		YAML::Node data = node["SurgSim::Framework::Scene"];

		if (data["SceneElements"].IsDefined())
		{
			auto sceneElements = data["SceneElements"].as<std::vector<std::shared_ptr<SceneElement>>>();

			std::for_each(sceneElements.begin(), sceneElements.end(),
						  [&](std::shared_ptr<SceneElement> element)
			{
				addSceneElement(element);
			});
		}
		result = true;
	}
	return result;
}

SurgSim::DataStructures::Groups<std::string, std::shared_ptr<SceneElement>>& Scene::getGroups()
{
	return m_groups;
}

std::shared_ptr<Component> Scene::getComponent(const std::string& elementName, const std::string& componentName) const
{
	std::shared_ptr<Component> result;
	auto element = getSceneElement(elementName);
	if (element != nullptr)
	{
		result = element->getComponent(componentName);
		if (result == nullptr)
		{
			SURGSIM_LOG_INFO(m_logger)
					<< "Could not find component '" << componentName << "' in Element '" << elementName << "'.";
		}
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << "Could not find element '" << elementName << "'.";
	}
	return result;
}

}; // namespace Framework
}; // namespace SurgSim


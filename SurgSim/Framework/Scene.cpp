// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest LLC.
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

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Log.h"

#include <utility>
#include <set>
#include <vector>
#include <boost/thread/locks.hpp>

#include <yaml-cpp/yaml.h>

namespace SurgSim
{
namespace Framework
{

Scene::Scene(std::weak_ptr<Runtime> runtime) :
	m_runtime(runtime),
	m_groups(new GroupsType())
{
	SURGSIM_ASSERT(!m_runtime.expired()) << "Can't create scene with empty runtime.";
}

Scene::~Scene()
{
	/// Clear out the groups references, this is needed because otherwise there will be circular references
	/// between the SceneElements and themselves therefore preventing the release of the the sceneelement
	/// instances, as it is shared with the sceneElements
	m_groups->clear();
}

void Scene::addSceneElement(std::shared_ptr<SceneElement> element)
{
	SURGSIM_ASSERT(!m_runtime.expired()) << "Runtime pointer is expired, cannot add SceneElement to Scene.";

	std::string name = element->getName();
	element->setScene(getSharedPtr());

	std::shared_ptr<Runtime> runtime = m_runtime.lock();
	element->setRuntime(runtime);

	if (element->initialize())
	{
		{
			boost::lock_guard<boost::mutex> lock(m_sceneElementsMutex);
			m_elements.push_back(element);
		}
		runtime->addSceneElement(element);
	}
}

void Scene::addSceneElements(std::vector<std::shared_ptr<SceneElement>> elements)
{
	for (auto element : elements)
	{
		addSceneElement(element);
	}
}

void Scene::removeSceneElement(std::shared_ptr<SceneElement> element)
{
	bool found;
	{
		boost::lock_guard<boost::mutex> lock(m_sceneElementsMutex);
		auto it = std::find(m_elements.begin(),  m_elements.end(), element);
		found = it != m_elements.end();
		if (found)
		{
			m_elements.erase(it);
		}
		else
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Framework/Scene"))
					<< "Could not find element '" << element->getName() << "' in Scene, unable to remove";
		}
	}
	if (found)
	{
		for (auto& component : element->getComponents())
		{
			element->removeComponent(component);
		}
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

std::shared_ptr<Scene::GroupsType> Scene::getGroups()
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
			SURGSIM_LOG_INFO(SurgSim::Framework::Logger::getLogger("Framework/Scene"))
					<< "Could not find component '" << componentName << "' in Element '" << elementName << "'.";
		}
	}
	else
	{
		SURGSIM_LOG_INFO(SurgSim::Framework::Logger::getLogger("Framework/Scene"))
				<< "Could not find element '" << elementName << "'.";
	}
	return result;
}

}; // namespace Framework
}; // namespace SurgSim


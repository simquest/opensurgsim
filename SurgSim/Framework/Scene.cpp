// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest LLC.
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
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Log.h"

#include <utility>
#include <boost/thread/locks.hpp>


namespace SurgSim
{
namespace Framework
{

Scene::Scene(std::weak_ptr<Runtime> runtime) :
	m_runtime(runtime)
{
	SURGSIM_ASSERT(!m_runtime.expired()) << "Can't create scene with empty runtime.";
}

Scene::~Scene()
{

}

void Scene::addSceneElement(std::shared_ptr<SceneElement> element)
{
	std::string name = element->getName();
	element->setScene(getSharedPtr());

	if (!m_runtime.expired())
	{
		std::shared_ptr<Runtime> runtime = m_runtime.lock();
		element->setRuntime(runtime);
		if (element->initialize())
		{
			boost::lock_guard<boost::mutex> lock(m_sceneElementsMutex);
			m_elements.insert(std::pair<std::string, std::shared_ptr<SceneElement>>(name, element));
			runtime->addSceneElement(element);
		}
	}
	else
	{
		SURGSIM_FAILURE() << "Runtime pointer is expired, cannot add SceneElement to Scene.";
	}

}

std::shared_ptr<SceneElement> Scene::getSceneElement(const std::string& name) const
{
	std::shared_ptr<SceneElement> result;	
	boost::lock_guard<boost::mutex> lock(m_sceneElementsMutex);
	auto found = m_elements.find(name);
	if (found != m_elements.end())
	{
		result = found->second;
	}
	return result;
}

std::shared_ptr<Runtime> Scene::getRuntime()
{
	return m_runtime.lock();
}

const std::multimap<std::string,std::shared_ptr<SceneElement>>& Scene::getSceneElements() const
{
	return m_elements;
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

}; // namespace Framework
}; // namespace SurgSim


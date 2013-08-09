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

#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Component.h>
#include <SurgSim/Framework/Log.h>

namespace SurgSim
{
namespace Framework
{

bool SceneElement::addComponent(std::shared_ptr<Component> component)
{
	bool result= false;

	SURGSIM_ASSERT(component != nullptr) << "Cannot add a nullptr as a component";

	if (m_components.find(component->getName()) == m_components.end())
	{
		component->setSceneElement(getSharedPtr());
		component->setScene(m_scene);

		m_components[component->getName()] = component;
		result = true;
	}
	else
	{
		SURGSIM_LOG_WARNING(m_runtime.lock()->getLogger("runtime")) <<
				"Component with name " << component->getName() <<
				" already exists on SceneElement " << getName() <<
				", did not add component";
	}
	return result;
}

bool SceneElement::removeComponent(std::shared_ptr<Component> component)
{
	return removeComponent(component->getName());
}

bool SceneElement::removeComponent(const std::string& name)
{
	bool result = false;
	if (m_components.find(name) != m_components.end())
	{
		size_t count = m_components.erase(name);
		result = (count == 1);
	}
	return result;
}

std::shared_ptr<Component> SceneElement::getComponent(const std::string& name) const
{
	std::shared_ptr<Component> result;

	auto findResult = m_components.find(name);
	if (findResult != m_components.end())
	{
		result = findResult->second;
	}
	return result;
}

bool SceneElement::initialize()
{
	m_isInitialized = doInitialize();
	return m_isInitialized;
}

bool SceneElement::wakeUp()
{
	m_isAwake = doWakeUp();
	return m_isAwake;
}

std::vector<std::shared_ptr<Component>> SceneElement::getComponents() const
{
	std::vector<std::shared_ptr<Component>> result(m_components.size());
	auto componentIt = m_components.begin();
	for (int i=0; componentIt != m_components.end(); ++componentIt, ++i)
	{
		result[i] = componentIt->second;
	}
	return result;
}

void SceneElement::setScene(std::weak_ptr<Scene> scene)
{
	m_scene = scene;

	auto it = std::begin(m_components);
	auto endIt = std::end(m_components);

	for ( ;  it != endIt;  ++it)
		{
			(it->second)->setScene(scene);
		}

}

std::shared_ptr<Scene> SceneElement::getScene()
{
	return m_scene.lock();
}

void SceneElement::setRuntime(std::shared_ptr<Runtime> runtime)
{
	m_runtime = runtime;
}

std::shared_ptr<Runtime> SceneElement::getRuntime()
{
	return m_runtime.lock();
}

std::shared_ptr<SceneElement> SceneElement::getSharedPtr()
{
	std::shared_ptr<SceneElement> result;
	try
	{
		result = shared_from_this();
	}
	catch (const std::exception&)
	{
		SURGSIM_FAILURE() << "SceneElement was not created as a shared_ptr";
	}
	return result;
}

}; // namespace Framework
}; // namespace SurgSim





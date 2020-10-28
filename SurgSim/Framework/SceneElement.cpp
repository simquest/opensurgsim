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

#include "SurgSim/Framework/SceneElement.h"

#include <yaml-cpp/yaml.h>

#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"


namespace SurgSim
{
namespace Framework
{

SceneElement::SceneElement(const std::string& name) :
	m_name(name),
	m_isInitialized(false),
	m_isActive(true)
{
	m_pose = std::make_shared<PoseComponent>("Pose");
	m_pose->setPose(Math::RigidTransform3d::Identity());
	m_components[m_pose->getName()] = m_pose;
}

SceneElement::~SceneElement()
{

}

bool SceneElement::addComponent(std::shared_ptr<Component> component)
{
	SURGSIM_ASSERT(nullptr != component) << "Cannot add a nullptr as a component";

	bool result = false;
	if (m_components.find(component->getName()) == m_components.end())
	{
		result = true;
		if (isInitialized())
		{
			auto runtime = getRuntime();
			SURGSIM_ASSERT(nullptr != runtime) << "Runtime cannot be expired when adding a component " << getName();
			result = initializeComponent(component);
			if (result)
			{
				runtime->addComponent(component);
			}
		}

		if (result)
		{
			try
			{
				// This will except if called during constructor
				// the this will be set in initialize()
				component->setSceneElement(shared_from_this());
			}
			catch (std::exception&)
			{
				// intentionally empty
			}
			m_components[component->getName()] = component;
		}
	}
	else
	{
		SURGSIM_LOG_WARNING(Logger::getLogger("runtime")) <<
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
	auto found = m_components.find(name);
	if (found != m_components.end())
	{
		if (isInitialized())
		{
			auto runtime = getRuntime();
			SURGSIM_ASSERT(nullptr != runtime) << "Runtime cannot be expired when removing a component " << getName();
			found->second->setLocalActive(false);
			runtime->removeComponent(found->second);
		}

		size_t count = m_components.erase(name);
		result = (count == 1);
	}
	return result;
}

void SceneElement::removeComponents()
{
	if (!m_components.empty())
	{
		auto runtime = getRuntime();
		SURGSIM_ASSERT(runtime) << "Runtime cannot be expired when removing components from " << getName();

		for (auto it = std::begin(m_components); it != std::end(m_components);  ++it)
		{
			it->second->setLocalActive(false);
			runtime->removeComponent(it->second);
		}

		m_components.clear();
	}
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
	SURGSIM_ASSERT(!m_isInitialized) << "Double initialization calls on SceneElement " << m_name;
	m_isInitialized = doInitialize();

	// If it did not happen in addComponent do it here
	m_pose->setSceneElement(getSharedPtr());

	if (m_isInitialized)
	{
		// initialize all components
		for (auto pair = std::begin(m_components); m_isInitialized && pair != std::end(m_components); ++pair)
		{
			m_isInitialized = initializeComponent(pair->second);
		}
	}

	return m_isInitialized;
}

bool SceneElement::initializeComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	component->setSceneElement(getSharedPtr());
	component->setScene(m_scene);
	return component->initialize(getRuntime());
}

std::string SceneElement::getName() const
{
	return m_name;
}

void SceneElement::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_pose->setPose(pose);
}

const SurgSim::Math::RigidTransform3d& SceneElement::getPose() const
{
	return m_pose->getPose();
}

std::shared_ptr<PoseComponent> SceneElement::getPoseComponent()
{
	return m_pose;
}

std::vector<std::shared_ptr<Component>> SceneElement::getComponents() const
{
	std::vector<std::shared_ptr<Component>> result(m_components.size());
	auto componentIt = m_components.begin();
	for (int i = 0; componentIt != m_components.end(); ++componentIt, ++i)
	{
		result[i] = componentIt->second;
	}
	return result;
}

boost::any SceneElement::getValue(const std::string& component, const std::string& property) const
{
	auto found = m_components.find(component);
	SURGSIM_ASSERT(found != m_components.end())
		<< "Component named " << component << " not found in SceneElement named " << getName()
		<< ". Cannot get " << property << " property.";
	return found->second->getValue(property);
}

void SceneElement::setValue(const std::string& component, const std::string& property, const boost::any& value)
{
	auto found = m_components.find(component);
	SURGSIM_ASSERT(found != m_components.end())
		<< "Component named " << component << " not found in SceneElement named " << getName()
		<< ". Cannot get " << property << " property.";
	found->second->setValue(property, value);
}

void SceneElement::setScene(std::weak_ptr<Scene> scene)
{
	m_scene = scene;
	for (auto it = std::begin(m_components); it != std::end(m_components);  ++it)
	{
		(it->second)->setScene(scene);
	}
}

std::shared_ptr<Scene> SceneElement::getScene()
{
	return m_scene.lock();
}

void SceneElement::setRuntime(std::weak_ptr<Runtime> runtime)
{
	m_runtime = runtime;
}

std::shared_ptr<Runtime> SceneElement::getRuntime()
{
	return m_runtime.lock();
}

bool SceneElement::isInitialized() const
{
	return m_isInitialized;
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
		SURGSIM_FAILURE() << "SceneElement was not created as a shared_ptr.";
	}
	return result;
}

void SceneElement::setName(const std::string& name)
{
	m_name = name;
}

YAML::Node SceneElement::encode(bool standalone) const
{
	YAML::Node data(YAML::NodeType::Map);
	data["Name"] = getName();
	data["IsActive"] = isActive();

	// Only encode groups when they are there, also encode them using the flow style i.e. with []
	if (m_groups.size() > 0)
	{
		data["Groups"] = m_groups;
		data["Groups"].SetStyle(YAML::EmitterStyle::Flow);
	}

	for (auto component = std::begin(m_components); component != std::end(m_components); ++component)
	{
		if (standalone)
		{
			data["Components"].push_back(*component->second);
		}
		else
		{
			data["Components"].push_back(component->second);
		}
	}
	YAML::Node node(YAML::NodeType::Map);
	node[getClassName()] = data;
	return node;
}

bool SceneElement::decode(const YAML::Node& node)
{
	SURGSIM_ASSERT(!isInitialized()) << "Should not call decode on a SceneElement that has already been initialized.";
	bool result = false;
	if (node.IsMap())
	{
		std::string className = node.begin()->first.as<std::string>();

		SURGSIM_ASSERT(className == getClassName()) << "Type in node does not match class, wanted <"
				<< className << ">" << " but this is a <" << getClassName() << ">.";

		YAML::Node data = node[getClassName()];

		SURGSIM_ASSERT(data.IsDefined() && !data.IsNull())
				<< "Content of node is empty, for class " << className << ". This is probably an indentation issue.";

		if (data["Name"].IsScalar())
		{
			m_name = data["Name"].as<std::string>();
		}

		if (data["IsActive"].IsDefined())
		{
			m_isActive = data["IsActive"].as<bool>();
		}

		if (data["Groups"].IsDefined())
		{

			m_groups = data["Groups"].as<std::unordered_set<std::string>>();
		}

		if (data["Components"].IsSequence())
		{
			for (auto nodeIt = data["Components"].begin(); nodeIt != data["Components"].end(); ++nodeIt)
			{
				if ("SurgSim::Framework::PoseComponent" == nodeIt->begin()->first.as<std::string>())
				{
					removeComponent(m_pose);
					m_pose = nodeIt->as<std::shared_ptr<PoseComponent>>();
					addComponent(m_pose);
				}
				else
				{
					try
					{
						addComponent(nodeIt->as<std::shared_ptr<Component>>());
					}
					catch (const YAML::Exception& e)
					{
						SURGSIM_FAILURE() << e.what() << "for " << std::endl << *nodeIt;
					}
				}
			}
			result = true;
		}
	}
	return result;
}

std::string SceneElement::getClassName() const
{
	// SURGSIM_FAILURE() << "SceneElement is abstract, this should not be called.";
	return "SurgSim::Framework::SceneElement";
}

void SceneElement::setActive(bool val)
{
	m_isActive = val;
}

bool SceneElement::isActive() const
{
	return m_isActive;
}

void SceneElement::addToGroup(const std::string& group)
{
	m_groups.insert(group);
	if (isInitialized())
	{
		getScene()->getGroups().add(group, getSharedPtr());
	}
}

void SceneElement::removeFromGroup(const std::string& group)
{
	m_groups.erase(group);
	if (isInitialized())
	{
		getScene()->getGroups().remove(group, getSharedPtr());
	}
}

void SceneElement::setGroups(const std::vector<std::string>& groups)
{
	if (isInitialized())
	{
		auto& groupings = getScene()->getGroups();
		groupings.remove(getSharedPtr());
		groupings.add(groups, getSharedPtr());
	}
	std::unordered_set<std::string> temp(groups.cbegin(), groups.cend());
	std::swap(m_groups, temp);
}

std::vector<std::string> SceneElement::getGroups() const
{
	return std::vector<std::string>(m_groups.cbegin(), m_groups.cend());
}

bool SceneElement::inGroup(const std::string& name)
{
	auto groups = getGroups();
	return (std::find(groups.begin(), groups.end(), name) != groups.end());
}

}; // namespace Framework
}; // namespace SurgSim

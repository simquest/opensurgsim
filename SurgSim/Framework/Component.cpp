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

#include "SurgSim/Framework/Component.h"

#include <boost/uuid/random_generator.hpp>

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Framework/SceneElement.h"

namespace SurgSim
{
namespace Framework
{

// Forward References
class Runtime;
class Scene;

Component::Component(const std::string& name) :
	m_name(name),
	m_uuid(boost::uuids::random_generator()()),
	m_didInit(false),
	m_didWakeUp(false),
	m_isInitialized(false),
	m_isAwake(false),
	m_isLocalActive(true)
{
	SURGSIM_ADD_RO_PROPERTY(Component, bool, IsActive, isActive);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Component, bool, IsLocalActive, isLocalActive, setLocalActive);
}

Component::~Component()
{
}

std::string Component::getName() const
{
	return m_name;
}

std::string Component::getFullName() const
{
	std::string name;
	const auto& sceneElement = getSceneElement();
	if (sceneElement != nullptr)
	{
		name = sceneElement->getName() + "/";
	}
	return name + m_name;
}

void Component::setName(const std::string& name)
{
	m_name = name;
}

bool Component::isInitialized() const
{
	return m_isInitialized;
}

bool Component::initialize(const std::weak_ptr<Runtime>& runtime)
{
	SURGSIM_ASSERT(!m_didInit) << "Double initialization called in component " << getFullName();
	m_runtime = runtime;

	m_didInit = true;
	m_isInitialized = doInitialize();

	return m_isInitialized;
}

bool Component::isAwake() const
{
	return m_isAwake;
}

bool Component::wakeUp()
{
	SURGSIM_ASSERT(! m_didWakeUp) << "Double wakeup called on component." << getFullName();
	SURGSIM_ASSERT(m_didInit) << "Component " << getFullName() << " was awoken without being initialized.";
	SURGSIM_ASSERT(m_isInitialized) << "Wakeup called even though initialization failed on component." << getFullName();

	m_didWakeUp = true;
	m_isAwake = doWakeUp();

	return m_isAwake;
}

void Component::retire()
{
	doRetire();
	m_isAwake = false;
}

void Component::doRetire()
{
	return;
}

void Component::setScene(std::weak_ptr<Scene> scene)
{
	m_scene = scene;
}

std::shared_ptr<Scene> Component::getScene()
{
	return m_scene.lock();
}

void Component::setSceneElement(std::weak_ptr<SceneElement> sceneElement)
{
	m_sceneElement = sceneElement;
}

std::shared_ptr<SceneElement> Component::getSceneElement()
{
	return m_sceneElement.lock();
}

std::shared_ptr<const SceneElement> Component::getSceneElement() const
{
	return m_sceneElement.lock();
}

std::shared_ptr<Runtime> Component::getRuntime() const
{
	return m_runtime.lock();
}

std::shared_ptr<const PoseComponent> Component::getPoseComponent() const
{
	SURGSIM_ASSERT(m_isInitialized) << "Can't access the pose component before initialization";
	return m_sceneElement.lock()->getPoseComponent();
}

std::shared_ptr<PoseComponent> Component::getPoseComponent()
{
	SURGSIM_ASSERT(m_isInitialized) << "Can't access the pose component before initialization";
	return m_sceneElement.lock()->getPoseComponent();
}

boost::uuids::uuid Component::getUuid() const
{
	return m_uuid;
}

std::string Component::getClassName() const
{
	SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "getClassName() called on Component base class, this is wrong" <<
			" in almost all cases, this means there is a class that does not have getClassName() defined.";
	return "SurgSim::Framework::Component";
}

std::shared_ptr<Component> Component::getSharedPtr()
{
	std::shared_ptr<Component> result;
	try
	{
		result = shared_from_this();
	}
	catch (const std::exception&)
	{
		SURGSIM_FAILURE() << "Component '" << getFullName() << "' not created as a shared_ptr.";
	}
	return result;
}

bool Component::isActive() const
{
	if (getSceneElement() != nullptr)
	{
		return getSceneElement()->isActive() && m_isLocalActive;
	}
	else
	{
		return m_isLocalActive;
	}
}

void Component::setLocalActive(bool val)
{
	m_isLocalActive = val;
}

bool Component::isLocalActive() const
{
	return m_isLocalActive;
}

}; // namespace Framework
}; // namespace SurgSim

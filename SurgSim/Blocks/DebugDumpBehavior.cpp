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

#include "SurgSim/Blocks/DebugDumpBehavior.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Input/InputComponent.h"

namespace SurgSim
{

namespace Blocks
{

DebugDumpBehavior::DebugDumpBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_keyPressedLastUpdate(false)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(DebugDumpBehavior, std::shared_ptr<SurgSim::Framework::Component>,
									  InputComponent, getInputComponent, setInputComponent);
}

DebugDumpBehavior::~DebugDumpBehavior()
{

}

void DebugDumpBehavior::setInputComponent(std::shared_ptr<SurgSim::Framework::Component> inputComponent)
{
	SURGSIM_ASSERT(nullptr != inputComponent) << "'inputComponent' cannot be 'nullptr'";

	m_inputComponent = SurgSim::Framework::checkAndConvert<SurgSim::Input::InputComponent>(
						   inputComponent, "SurgSim::Input::InputComponent");
}

std::shared_ptr<SurgSim::Input::InputComponent> DebugDumpBehavior::getInputComponent() const
{
	return m_inputComponent;
}

void DebugDumpBehavior::update(double dt)
{
	SurgSim::DataStructures::DataGroup dataGroup;

	if (m_inputComponent == nullptr)
	{
		return;
	}

	m_inputComponent->getData(&dataGroup);

	int key = SurgSim::Device::KeyCode::NONE;
	if (dataGroup.integers().get("key", &key))
	{

		if (!m_keyPressedLastUpdate)
		{
			switch (key)
			{
				case SurgSim::Device::KeyCode::KEY_F1:
					{
						auto manager = m_manager.lock();
						if (manager != nullptr)
						{
							manager->dumpDebugInfo();
						}
					}
					break;
				case SurgSim::Device::KeyCode::KEY_F2:
					{
						auto scene = getScene();
						for (const auto& element : scene->getSceneElements())
						{
							std::cout << element->getName()
									  << ": "
									  << (element->isActive() ?  "active" : "inactive") << std::endl;
							for (const auto& component : element->getComponents())
							{
								std::cout << "    "
										  << component->getName() << ": "
										  << (component->isLocalActive() ?  "active" : "inactive") <<
										  std::endl;
							}
						}
					}
					break;
				case SurgSim::Device::KeyCode::KEY_F3:
					{
						auto scene = getScene();
						for (const auto& element : scene->getSceneElements())
						{
							std::cout << element->getName() << std::endl;
							for (const auto& component : element->getComponents<SurgSim::Graphics::Representation>())
							{
								std::cout << "    " << component->getName()
										  << ": " << (component->isGeneratingTangents() ?  "Tangents" : "NoTangents")
										  << std::endl;
							}

						}
					}
					break;
				default:
					break;
			}
		}
		m_keyPressedLastUpdate = (SurgSim::Device::KeyCode::NONE != key);
	}
}

bool DebugDumpBehavior::doInitialize()
{
	auto managers = getRuntime()->getManagers();
	for (const auto& manager : managers)
	{
		auto shared = manager.lock();
		auto graphics = std::dynamic_pointer_cast<SurgSim::Graphics::OsgManager>(shared);
		if (graphics != nullptr)
		{
			m_manager = graphics;
			break;
		}
	}
	SURGSIM_ASSERT(! m_manager.expired()) << "Can't find graphics manager.";
	return true;
}

bool DebugDumpBehavior::doWakeUp()
{
	bool result = true;
	if (nullptr == m_inputComponent)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__ <<
				"KeyboardTogglesComponentBehavior " << getName() << " does not have an Input Component.";
		result = false;
	}
	return result;
}

}
}
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

#include "SurgSim/Framework/BehaviorManager.h"

#include <memory>
#include <vector>

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Framework/Runtime.h"


SurgSim::Framework::BehaviorManager::BehaviorManager() : ComponentManager("Behavior Manager")
{
	m_logger = SurgSim::Framework::Logger::createConsoleLogger(getName());
}

SurgSim::Framework::BehaviorManager::~BehaviorManager()
{

}

bool SurgSim::Framework::BehaviorManager::doInitialize()
{
	m_logger = getRuntime()->getLogger(getName());
	return true;
}

bool SurgSim::Framework::BehaviorManager::doStartUp()
{
	return true;
}

bool SurgSim::Framework::BehaviorManager::addComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	bool result = true;
	std::shared_ptr<Behavior> behavior = std::dynamic_pointer_cast<Behavior>(component);
	if (behavior != nullptr)
	{
		if (find(m_behaviors.begin(), m_behaviors.end(),behavior) == m_behaviors.end())
		{
			m_behaviors.push_back(behavior);
			SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Added behavior " << behavior->getName();
		}
		else
		{
			result = false;
			SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Duplicate behavior " << behavior->getName();
		}
	}
	return result;
}

bool SurgSim::Framework::BehaviorManager::removeComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	bool result = false;
	std::shared_ptr<Behavior> behavior = std::dynamic_pointer_cast<Behavior>(component);
	if (behavior != nullptr && m_behaviors.size() != 0)
	{
		auto found = std::find(m_behaviors.begin(), m_behaviors.end(), behavior);
		if (found != m_behaviors.end())
		{
			m_behaviors.erase(found);
			SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Removed behavior " << behavior->getName();
			result = true;
		}
		else
		{
			SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Unable to remove behavior " << behavior->getName()
									   << ". Not found.";
		}
	}
	return result;
}

bool SurgSim::Framework::BehaviorManager::doUpdate(double dt)
{
	auto endIt = m_behaviors.end();
	auto it = m_behaviors.begin();
	for (;it != endIt; ++it)
	{
		(*it)->update(dt);
	}
	return true;
}



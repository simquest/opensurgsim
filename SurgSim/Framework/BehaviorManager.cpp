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


namespace SurgSim
{
namespace Framework
{

BehaviorManager::BehaviorManager() : ComponentManager("Behavior Manager")
{
	m_logger = SurgSim::Framework::Logger::getLogger(getName());
}

BehaviorManager::~BehaviorManager()
{

}

bool BehaviorManager::doInitialize()
{
	return true;
}

bool BehaviorManager::doStartUp()
{
	return true;
}

bool BehaviorManager::executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	return tryAddComponent(component, &m_behaviors) != nullptr;
}

bool BehaviorManager::executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	return tryRemoveComponent(component, &m_behaviors);
}

bool SurgSim::Framework::BehaviorManager::doUpdate(double dt)
{
	// Add all components that came in before the last update
	processComponents();

	auto it = std::begin(m_behaviors);
	auto endIt = std::end(m_behaviors);
	for ( ;  it != endIt;  ++it)
	{
		(*it)->update(dt);
	}
	return true;
}

}; // namespace Framework
}; // namespace SurgSim




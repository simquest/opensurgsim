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

#include <SurgSim/Framework/ComponentManager.h>
#include <SurgSim/Framework/Component.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/Runtime.h>

#include <boost/thread/locks.hpp>

namespace SurgSim
{
namespace Framework
{

ComponentManager::ComponentManager(const std::string& name /*= "Unknown Component Manager"*/) :
	BasicThread(name), m_logger(Logger::getLogger(name))
{
}

ComponentManager::~ComponentManager()
{

}

void ComponentManager::setRuntime(std::shared_ptr<Runtime> val)
{
	m_runtime = val;
}

bool ComponentManager::enqueueAddComponent(const std::shared_ptr<Component>& component)
{
	boost::lock_guard<boost::mutex> lock(m_componentMutex);
	m_componentAdditions.push_back(component);
	return true;
}

bool ComponentManager::enqueueRemoveComponent(const std::shared_ptr<Component>& component)
{
	boost::lock_guard<boost::mutex> lock(m_componentMutex);
	m_componentRemovals.push_back(component);
	return true;
}

void ComponentManager::processComponents()
{
	// Please note that the implementation of this function needs to mirror the executeInitialization() function
	// this is called from within the update() function, the other is called at startup
	std::vector<std::shared_ptr<Component>> inflightAdditions;
	std::vector<std::shared_ptr<Component>> inflightRemovals;
	std::vector<std::shared_ptr<Component>> actualAdditions;

	copyScheduledComponents(&inflightAdditions, &inflightRemovals);
	actualAdditions.reserve(inflightAdditions.size());

	if (!inflightAdditions.empty())
	{
		addAndIntializeComponents(std::begin(inflightAdditions), std::end(inflightAdditions), &actualAdditions);
		wakeUpComponents(std::begin(actualAdditions), std::end(actualAdditions));
	}

	if (!inflightRemovals.empty())
	{
		removeComponents(std::begin(inflightRemovals), std::end(inflightRemovals));
	}
}

bool ComponentManager::executeInitialization()
{
	// Please note that the implementation of this function needs to mirror processComponents()
	// this function is called at startup whereas the other is called during the update call.

	// Call BasicThread initialize to do the initialize and startup call
	bool success = BasicThread::executeInitialization();
	if (! success )
	{
		return success;
	}

	// Now Initialize and and wakeup all the components
	std::vector<std::shared_ptr<Component>> inflightAdditions;
	std::vector<std::shared_ptr<Component>> inflightRemovals;
	std::vector<std::shared_ptr<Component>> actualAdditions;

	copyScheduledComponents(&inflightAdditions, &inflightRemovals);
	actualAdditions.reserve(inflightAdditions.size());

	if (! inflightAdditions.empty())
	{
		addAndIntializeComponents(std::begin(inflightAdditions), std::end(inflightAdditions), &actualAdditions);
	}

	success = waitForBarrier(success);
	if (! success)
	{
		return success;
	}

	if (! inflightAdditions.empty())
	{
		wakeUpComponents(std::begin(actualAdditions), std::end(actualAdditions));
	}

	if (! inflightRemovals.empty())
	{
		removeComponents(std::begin(inflightRemovals), std::end(inflightRemovals));
	}

	// Wait for SceneElement WakeUp, the last in the sequence
	success = waitForBarrier(success);

	success = waitForBarrier(success);

	return success;
}

void ComponentManager::copyScheduledComponents(
	std::vector<std::shared_ptr<Component>>* inflightAdditions,
	std::vector<std::shared_ptr<Component>>* inflightRemovals
)
{

	// Lock for any more additions or removals and then copy to local storage
	// this will insulate us from the actual add or remove call taking longer than it should
	boost::lock_guard<boost::mutex> lock(m_componentMutex);
	*inflightAdditions = std::move(m_componentAdditions);
	m_componentAdditions.clear();

	*inflightRemovals = std::move(m_componentRemovals);
	m_componentRemovals.clear();
}

void ComponentManager::removeComponents(const std::vector<std::shared_ptr<Component>>::const_iterator& beginIt,
										const std::vector<std::shared_ptr<Component>>::const_iterator& endIt)
{
	for(auto it = beginIt; it != endIt; ++it)
	{
		executeRemovals(*it);
	}
}

void ComponentManager::addAndIntializeComponents(
	const std::vector<std::shared_ptr<Component>>::const_iterator& beginIt,
	const std::vector<std::shared_ptr<Component>>::const_iterator& endIt,
	std::vector<std::shared_ptr<Component>>* actualAdditions)
{
	// Add All Components to the internal storage
	for(auto it = beginIt; it != endIt; ++it)
	{
		if (executeAdditions(*it) && (*it)->initialize(std::move(getRuntime())))
		{
			actualAdditions->push_back(*it);
		}
	}
}

void ComponentManager::wakeUpComponents(const std::vector<std::shared_ptr<Component>>::const_iterator& beginIt,
	const std::vector<std::shared_ptr<Component>>::const_iterator& endIt)
{
	for(auto it = beginIt; it != endIt; ++it)
	{
		if ( (*it)->isInitialized() && !(*it)->isAwake())
		{
			if ( !(*it)->wakeUp())
			{
				SURGSIM_LOG_WARNING(m_logger) << "Failed to wake up component " << (*it)->getName() << " in manager " <<
					getName() << ". Component was not added to the manager!";
				executeRemovals(*it);
			}
		}
	}
}


bool ComponentManager::executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	return tryAddComponent(component, &m_behaviors) != nullptr;
}

bool ComponentManager::executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	return tryRemoveComponent(component, &m_behaviors);
}

bool ComponentManager::doUpdate(double dt)
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

}; // Framework
}; // SurgSim

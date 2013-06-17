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

#include <boost/thread/lock_guard.hpp>

namespace SurgSim
{
namespace Framework
{

ComponentManager::ComponentManager(const std::string& name /*= "Unknown Component Manager"*/) :
	m_logger (Logger::createConsoleLogger(name)), BasicThread(name)
{
}

ComponentManager::~ComponentManager()
{

}

void ComponentManager::setRuntime(std::shared_ptr<Runtime> val)
{
	m_runtime = val;

	// Now pull the shared logger out of the runtime
	m_logger = val->getLogger(getName());
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
	std::vector<std::shared_ptr<Component>> inflightAdditions;
	std::vector<std::shared_ptr<Component>> inflightRemovals;

	copyScheduledComponents(&inflightAdditions, &inflightRemovals);

	if (!inflightAdditions.empty())
	{
		auto inflightBegin = std::begin(inflightAdditions);
		auto inflightEnd = std::end(inflightAdditions);

		addAndIntializeComponents(inflightBegin, inflightEnd);
		wakeUpComponents(inflightBegin, inflightEnd);
	}

	if (!inflightRemovals.empty())
	{
		removeComponents(std::begin(inflightRemovals), std::end(inflightRemovals));
	}
}

bool ComponentManager::executeInitialization()
{
	// Call BasicThread initialize to do the initialize and startup call
	bool success = BasicThread::executeInitialization();
	if (! success )
	{
		return success;
	}

	// Now Initialize and and wakeup all the components
	std::vector<std::shared_ptr<Component>> inflightAdditions;
	std::vector<std::shared_ptr<Component>> inflightRemovals;
	
	copyScheduledComponents(&inflightAdditions, &inflightRemovals);

	auto inflightBegin = std::begin(inflightAdditions);
	auto inflightEnd = std::end(inflightAdditions);
	
	if (! inflightAdditions.empty())
	{
		addAndIntializeComponents(inflightBegin, inflightEnd);
	}

	success = waitForBarrier(success);
	if (! success)
	{
		return success;
	}

	if (! inflightAdditions.empty())
	{
		wakeUpComponents(inflightBegin, inflightEnd);
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

void ComponentManager::addAndIntializeComponents(const std::vector<std::shared_ptr<Component>>::const_iterator& beginIt,
	const std::vector<std::shared_ptr<Component>>::const_iterator& endIt)
{
	// Add All Components to the internal storage
	for(auto it = beginIt; it != endIt; ++it)
	{
		if (executeAdditions(*it))
		{
			(*it)->initialize(std::move(getRuntime()));
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

}; // Framework
}; // SurgSim

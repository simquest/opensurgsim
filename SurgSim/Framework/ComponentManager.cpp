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

bool ComponentManager::addComponent(const std::shared_ptr<Component>& component)
{
	boost::lock_guard<boost::mutex> lock(m_componentMutex);
	m_componentAdditions.push_back(component);
	return true;
}

bool ComponentManager::removeComponent(const std::shared_ptr<Component>& component)
{
	boost::lock_guard<boost::mutex> lock(m_componentMutex);
	m_componentRemovals.push_back(component);
	return true;
}

void ComponentManager::processComponents()
{
	copyScheduledComponents();

	if (!m_inflightAdditions.empty())
	{
		auto inflightBegin = std::begin(m_inflightAdditions);
		auto inflightEnd = std::end(m_inflightAdditions);

		initializeComponents(inflightBegin, inflightEnd);
		wakeUpComponents(inflightBegin, inflightEnd);
		m_inflightAdditions.clear();
	}

	if (!m_inflightRemovals.empty())
	{
		removeComponents(std::begin(m_inflightRemovals), std::end(m_inflightRemovals));
		m_inflightRemovals.clear();
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
	copyScheduledComponents();
	auto inflightBegin = std::begin(m_inflightAdditions);
	auto inflightEnd = std::end(m_inflightAdditions);

	if (!m_inflightAdditions.empty())
	{
		initializeComponents(inflightBegin, inflightEnd);
		wakeUpComponents(inflightBegin, inflightEnd);
		m_inflightAdditions.clear();
	}

	success = waitForBarrier(success);

	if (! success)
	{
		return success;
	}

	if (!m_inflightAdditions.empty())
	{
		removeComponents(std::begin(m_inflightRemovals), std::end(m_inflightRemovals));
		m_inflightRemovals.clear();
	}
	success = waitForBarrier(success);

	// Wait for SceneElement WakeUp, the last in the sequence
	success = waitForBarrier(success);
	return success;
}

void ComponentManager::copyScheduledComponents()
{
	m_inflightAdditions.clear();
	m_inflightRemovals.clear();

	// Lock for any more additions or removals and then copy to local storage
	// this will insulate us from the actual add or remove call taking longer than it should
	boost::lock_guard<boost::mutex> lock(m_componentMutex);
	m_inflightAdditions.resize(m_componentAdditions.size());
	std::copy(m_componentAdditions.begin(), m_componentAdditions.end(), m_inflightAdditions.begin());
	m_componentAdditions.clear();

	m_inflightRemovals.resize(m_componentRemovals.size());
	std::copy(m_componentRemovals.begin(), m_componentRemovals.end(), m_inflightRemovals.begin());
	m_componentRemovals.clear();
}

void ComponentManager::removeComponents(const std::vector<std::shared_ptr<Component>>::const_iterator& beginIt,
										const std::vector<std::shared_ptr<Component>>::const_iterator& endIt)
{
	for(auto it = beginIt; it != endIt; ++it)
	{
		doRemoveComponent(*it);
	}
}

void ComponentManager::initializeComponents(const std::vector<std::shared_ptr<Component>>::const_iterator& beginIt,
	const std::vector<std::shared_ptr<Component>>::const_iterator& endIt)
{
	// Add All Components to the internal storage
	for(auto it = beginIt; it != endIt; ++it)
	{
		if (doAddComponent(*it))
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
				doRemoveComponent(*it);
			}
		}
	}
}

}; // Framework
}; // SurgSim

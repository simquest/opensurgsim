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
	std::vector<std::shared_ptr<Component>> inflightAdditions;
	std::vector<std::shared_ptr<Component>> inflightRemovals;

	{
		// Lock for any more additions or removals and then copy to local storage
		// this will insulate us from the actual add or remove call taking longer than it should
		boost::lock_guard<boost::mutex> lock(m_componentMutex);
		inflightAdditions.resize(m_componentAdditions.size());
		std::copy(m_componentAdditions.begin(), m_componentAdditions.end(), inflightAdditions.begin());		
		m_componentAdditions.clear();

		inflightRemovals.resize(m_componentRemovals.size());
		std::copy(m_componentRemovals.begin(), m_componentRemovals.end(), inflightRemovals.begin());		
		m_componentRemovals.clear();
	}
	
	auto inflightBegin = std::begin(inflightAdditions);
	auto inflightEnd = std::end(inflightAdditions);

	// Add All Components to the internal storage
	for(auto it = inflightBegin; it != inflightEnd; ++it)
	{
		if (doAddComponent(*it))
		{
			(*it)->initialize(std::move(getRuntime()));
		}
	}

	// And again to wake all the components up, only the components that were successfully initialized
	// get the wakeup call, check for isAwake because there to catch multiple versions of the same component
	// from being awoken more than once
	for(auto it = inflightBegin; it != inflightEnd; ++it)
	{
		if ( (*it)->isInitialized() && !(*it)->isAwake())
		{
			(*it)->wakeUp();
		}
	}

	// This is optional but clean up if anything did not wake up correctly ... 
	for(auto it = inflightBegin; it != inflightEnd; ++it)
	{
		if ((*it)->isInitialized() && !(*it)->isAwake())
		{
			doRemoveComponent(*it);
		}
	}

	for(auto it = std::begin(inflightRemovals); it != std::end(inflightRemovals); ++it)
	{
		doRemoveComponent(*it);
	}
}

}; // Framework
}; // SurgSim

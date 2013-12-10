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

#include "SurgSim/Framework/TransferPropertiesBehavior.h"

#include <boost/thread/lock_guard.hpp>

namespace SurgSim
{
namespace Framework
{

TransferPropertiesBehavior::TransferPropertiesBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_targetManager(MANAGER_TYPE_BEHAVIOR)
{

}

TransferPropertiesBehavior::~TransferPropertiesBehavior()
{

}

void TransferPropertiesBehavior::update(double dt)
{
	{
		boost::lock_guard<boost::mutex>lock(m_incomingMutex);
		m_connections.insert(m_connections.end(), m_incomingConnections.begin(), m_incomingConnections.end());
		m_incomingConnections.clear();
	}

	for (auto it = std::begin(m_connections); it != std::end(m_connections); ++it)
	{
		if (!it->first.second.expired() && !it->second.second.expired())
		{
			auto source = it->first.second.lock();
			auto target = it->second.second.lock();
			target->setValue(it->second.first,source->getValue(it->first.first));
		}
	}
}

bool TransferPropertiesBehavior::connect(
	std::shared_ptr<SurgSim::Framework::Accessible> source,
	const std::string& sourcePropertyName,
	std::shared_ptr<SurgSim::Framework::Accessible> target,				   
	const std::string& targetPropertyName)
{

	// Early outs
	if (source == nullptr || target == nullptr)
	{
		return false;
	}

	if (source == target && sourcePropertyName == targetPropertyName)
	{
		return false;
	}

	if (! source->isReadable(sourcePropertyName) || !target->isWriteable(targetPropertyName))
	{
		return false;
	}

	// \note HS-2013-nov-26 should also check for existing properties and matching types here

	Property sourceProperty = std::make_pair(sourcePropertyName, source);
	Property targetProperty = std::make_pair(targetPropertyName, target);

	auto entry = std::make_pair(std::move(sourceProperty), std::move(targetProperty));

	boost::lock_guard<boost::mutex>lock(m_incomingMutex);
	m_incomingConnections.push_back(std::move(entry));

	return true;
}

bool TransferPropertiesBehavior::doInitialize()
{
	return true;
}

bool TransferPropertiesBehavior::doWakeUp()
{
	// Do an update step to initialize all the values
	update(0.0);
	return true;
}

void TransferPropertiesBehavior::setTargetManagerType(int managerType)
{
	SURGSIM_ASSERT(managerType > 0 && managerType < MANAGER_TYPE_COUNT) << "Invalid manager type.";
	SURGSIM_ASSERT(!isInitialized()) << "Cannot change the manager type after initialization.";
	m_targetManager = managerType;
}

int TransferPropertiesBehavior::getTargetManagerType() const
{
	return m_targetManager;
}

}
}

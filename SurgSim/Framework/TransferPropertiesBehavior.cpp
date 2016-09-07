// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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
#include <boost/any.hpp>

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
		boost::lock_guard<boost::mutex> lock(m_incomingMutex);
		m_connections.insert(m_connections.end(), m_incomingConnections.begin(), m_incomingConnections.end());
		m_incomingConnections.clear();
	}

	for (auto it = std::begin(m_connections); it != std::end(m_connections); ++it)
	{
		if (!it->first.accessible.expired() && !it->second.accessible.expired())
		{
			auto source = it->first.accessible.lock();
			auto target = it->second.accessible.lock();
			target->setValue(it->second.name, source->getValue(it->first.name));
		}
	}
}

bool TransferPropertiesBehavior::connect(
	std::shared_ptr<SurgSim::Framework::Accessible> sourceAccessible,
	const std::string& sourcePropertyName,
	std::shared_ptr<SurgSim::Framework::Accessible> targetAccessible,
	const std::string& targetPropertyName)
{
	SURGSIM_ASSERT(sourceAccessible != nullptr && targetAccessible != nullptr) <<
			"Accessibles cannot be nullptr";

	Property source = {sourceAccessible, sourcePropertyName};
	Property target = {targetAccessible, targetPropertyName};

	return connect(source, target);
}

bool TransferPropertiesBehavior::connect(const Property& source, const Property& target)
{
	// Early outs
	if (source.accessible.expired() || target.accessible.expired())
	{
		return false;
	}

	auto sharedSource = source.accessible.lock();
	auto sharedTarget = target.accessible.lock();

	SURGSIM_ASSERT(sharedSource != sharedTarget || source.name != target.name)
			<< "Cannot Read/Write with exactly the same property and object.";

	SURGSIM_ASSERT(sharedSource->isReadable(source.name))
			<< "Source does not have a readable property called <" << source.name << ">.";

	SURGSIM_ASSERT(sharedTarget->isWriteable(target.name))
			<< "Target does not have a writeable property called <" << target.name << ">.";

	// \note HS-2013-nov-26 should also that the type of the output can be converted to the input

	auto entry = std::make_pair(source, target);

	boost::lock_guard<boost::mutex> lock(m_incomingMutex);
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
	SURGSIM_ASSERT(managerType >= 0 && managerType < MANAGER_TYPE_COUNT) << "Invalid manager type.";
	SURGSIM_ASSERT(!isInitialized()) << "Cannot change the manager type after initialization.";
	m_targetManager = managerType;
}

int TransferPropertiesBehavior::getTargetManagerType() const
{
	return m_targetManager;
}

}
}

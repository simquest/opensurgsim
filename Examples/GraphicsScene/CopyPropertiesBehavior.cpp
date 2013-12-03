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

#include <Examples/GraphicsScene/CopyPropertiesBehavior.h>


CopyPropertiesBehavior::CopyPropertiesBehavior(const std::string& name) : SurgSim::Framework::Behavior(name)
{

}

CopyPropertiesBehavior::~CopyPropertiesBehavior()
{

}

void CopyPropertiesBehavior::update(double dt)
{
	for (auto it = std::begin(m_properties); it != std::end(m_properties); ++it)
	{
		auto source = it->first.second.lock();
		auto target = it->second.second.lock();
		if (source != nullptr && target !=nullptr)
		{
			target->setValue(it->second.first,source->getValue(it->first.first));
		}
	}
}

bool CopyPropertiesBehavior::addConnection(const std::string& sourcePropertyName,
										   std::shared_ptr<SurgSim::Framework::Accessible> source,
										   const std::string& targetPropertyName,
										   std::shared_ptr<SurgSim::Framework::Accessible> target)
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

	// \note HS-2013-nov-26 should also check for existing properties and matching types here 

	Property sourceProperty = std::make_pair(sourcePropertyName, source);
	Property targetProperty = std::make_pair(targetPropertyName, target);

	auto entry = std::make_pair(std::move(sourceProperty), std::move(targetProperty));

	// Probably need to protect this, we might be manipulating in later on
	// #threadsafety
	m_properties.push_back(std::move(entry));

	return true;
}

bool CopyPropertiesBehavior::doInitialize()
{
	return true;
}

bool CopyPropertiesBehavior::doWakeUp()
{
	// Do an update step to initialize all the values
	update(0.0);
	return true;
}


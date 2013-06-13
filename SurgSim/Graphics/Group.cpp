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

#include <SurgSim/Graphics/Group.h>

#include <SurgSim/Graphics/Representation.h>

using SurgSim::Graphics::Representation;
using SurgSim::Graphics::Group;

Group::Group(const std::string& name) : SurgSim::Framework::Component(name)
{
}
Group::~Group()
{
}

bool Group::add(std::shared_ptr<Representation> representation)
{
	bool result = false;
	if (std::find(m_representations.begin(), m_representations.end(), representation) == m_representations.end())
	{
		m_representations.push_back(representation);
		result = true;
	}
	return result;
}

bool Group::append(std::shared_ptr<Group> group)
{
	bool result = true;
	const std::vector<std::shared_ptr<Representation>>& members = group->getMembers();
	for (auto it = members.begin(); it != members.end(); ++it)
	{
		if (! add(*it))
		{
			result = false;
		}
	}
	return result;
}

bool Group::remove(std::shared_ptr<Representation> representation)
{
	bool result = false;
	auto it = std::find(m_representations.begin(), m_representations.end(), representation);
	if (it != m_representations.end())
	{
		m_representations.erase(it);
		result = true;
	}
	return result;
}

void Group::clear()
{
	m_representations.clear();
}

bool Group::doInitialize()
{
	return true;
}

bool Group::doWakeUp()
{
	return true;
}

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

#include "Group.h"

#include <SurgSim/Graphics/Actor.h>

using SurgSim::Graphics::Actor;
using SurgSim::Graphics::Group;

Group::Group(const std::string& name) : Representation(name)
{
}
Group::~Group()
{
}

bool Group::addActor(std::shared_ptr<Actor> actor)
{
	bool result = false;
	if (std::find(m_actors.begin(), m_actors.end(), actor) == m_actors.end())
	{
		m_actors.push_back(actor);
		result = true;
	}
	return result;
}

bool Group::removeActor(std::shared_ptr<Actor> actor)
{
	bool result = false;
	auto it = std::find(m_actors.begin(), m_actors.end(), actor);
	if (it != m_actors.end())
	{
		m_actors.erase(it);
		result = true;
	}
	return result;
}

void Group::clearActors()
{
	m_actors.clear();
}

bool Group::addGroup(std::shared_ptr<Group> group)
{
	bool result = false;
	if (std::find(m_groups.begin(), m_groups.end(), group) == m_groups.end())
	{
		m_groups.push_back(group);
		result = true;
	}
	return result;
}

bool Group::removeGroup(std::shared_ptr<Group> group)
{
	bool result = false;
	auto it = std::find(m_groups.begin(), m_groups.end(), group);
	if (it != m_groups.end())
	{
		m_groups.erase(it);
		result = true;
	}
	return result;
}

void Group::clearGroups()
{
	m_groups.clear();
}

void Group::clear()
{
	clearActors();
	clearGroups();
}
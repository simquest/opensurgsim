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

#include <SurgSim/Graphics/Manager.h>

#include <SurgSim/Framework/Log.h>
#include <SurgSim/Graphics/Representation.h>
#include <SurgSim/Graphics/Camera.h>
#include <SurgSim/Graphics/Group.h>
#include <SurgSim/Graphics/View.h>

using SurgSim::Graphics::Representation;
using SurgSim::Graphics::Group;
using SurgSim::Graphics::Manager;
using SurgSim::Graphics::View;

Manager::Manager() : ComponentManager("Graphics Manager")
{
	m_logger = SurgSim::Framework::Logger::createConsoleLogger(getName());
}

Manager::~Manager()
{
}

bool Manager::executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	bool result = false;
	std::shared_ptr<Representation> representation = std::dynamic_pointer_cast<Representation>(component);
	if (representation != nullptr)
	{
		result = removeRepresentation(representation);
	}
	std::shared_ptr<Group> group = std::dynamic_pointer_cast<Group>(component);
	if (group != nullptr)
	{
		result = removeGroup(group);
	}
	std::shared_ptr<View> view = std::dynamic_pointer_cast<View>(component);
	if (view != nullptr)
	{
		result = removeView(view);
	}
	return result;
}

bool Manager::executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	bool result = false;
	std::shared_ptr<Representation> representation = std::dynamic_pointer_cast<Representation>(component);
	if (representation != nullptr)
	{
		result = addRepresentation(representation);
	}
	std::shared_ptr<Group> group = std::dynamic_pointer_cast<Group>(component);
	if (group != nullptr)
	{
		result = addGroup(group);
	}
	std::shared_ptr<View> view = std::dynamic_pointer_cast<View>(component);
	if (view != nullptr)
	{
		result = addView(view);
	}
	return result;
}

bool Manager::addRepresentation(std::shared_ptr<Representation> representation)
{
	bool result = false;
	if (std::find(m_representations.begin(), m_representations.end(), representation) == m_representations.end())
	{
		m_representations.push_back(representation);
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Added representation " << representation->getName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Duplicate representation " << representation->getName();
	}
	return result;
}
bool Manager::addGroup(std::shared_ptr<Group> group)
{
	bool result = false;
	if (std::find(m_groups.begin(), m_groups.end(), group) == m_groups.end())
	{
		m_groups.push_back(group);
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Added group " << group->getName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Duplicate group " << group->getName();
	}
	return result;
}
bool Manager::addView(std::shared_ptr<View> view)
{
	bool result = false;
	if (std::find(m_views.begin(), m_views.end(), view) == m_views.end())
	{
		m_views.push_back(view);
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Added view " << view->getName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Duplicate view " << view->getName();
	}
	return result;
}

bool Manager::removeRepresentation(std::shared_ptr<Representation> representation)
{
	bool result = false;
	auto it = std::find(m_representations.begin(), m_representations.end(), representation);
	if (it != m_representations.end())
	{
		m_representations.erase(it);
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Removed representation " << representation->getName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Representation not found " << representation->getName();
	}
	return result;
}
bool Manager::removeGroup(std::shared_ptr<Group> group)
{
	bool result = false;
	auto it = std::find(m_groups.begin(), m_groups.end(), group);
	if (it != m_groups.end())
	{
		m_groups.erase(it);
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Removed group " << group->getName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Group not found " << group->getName();
	}
	return result;
}
bool Manager::removeView(std::shared_ptr<View> view)
{
	bool result = false;
	auto it = std::find(m_views.begin(), m_views.end(), view);
	if (it != m_views.end())
	{
		m_views.erase(it);
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " Removed view " << view->getName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << __FUNCTION__ << " View not found " << view->getName();
	}
	return result;
}

bool Manager::doInitialize()
{
	return true;
}

bool Manager::doStartUp()
{
	return true;
}

bool Manager::doUpdate(double dt)
{
	processComponents();

	for (auto it = m_representations.begin(); it != m_representations.end(); ++it)
	{
		(*it)->update(dt);
	}
	for (auto it = m_views.begin(); it != m_views.end(); ++it)
	{
		(*it)->update(dt);
	}
	return true;
}
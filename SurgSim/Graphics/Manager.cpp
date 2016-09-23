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

#include "SurgSim/Graphics/Manager.h"

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Graphics/Light.h"
#include "SurgSim/Graphics/Group.h"
#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Graphics/View.h"

using SurgSim::Graphics::Camera;
using SurgSim::Graphics::Group;
using SurgSim::Graphics::Manager;
using SurgSim::Graphics::Representation;
using SurgSim::Graphics::View;

Manager::Manager() : ComponentManager("Graphics Manager")
{
	setRate(60.0);
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

		// Check all the groups that are requested for this representation, fetch them and
		// add this representation
		std::vector<std::string> requestedGroups = representation->getGroupReferences();
		for (auto groupName = std::begin(requestedGroups); groupName != std::end(requestedGroups); ++groupName)
		{
			auto group = getOrCreateGroup(*groupName);
			group->add(representation);
		}

		// Additionally for a camera create or fetch the RenderGroup
		auto camera = std::dynamic_pointer_cast<Camera>(representation);
		if (camera != nullptr)
		{
			std::vector<std::shared_ptr<Group>> groups;
			for (auto reference : camera->getRenderGroupReferences())
			{
				groups.push_back(getOrCreateGroup(reference));
			}
			camera->setRenderGroups(groups);
		}

		auto light = std::dynamic_pointer_cast<Light>(representation);
		if (light != nullptr)
		{
			light->setGroup(getOrCreateGroup(light->getLightGroupReference()));
		}

		SURGSIM_LOG_INFO(m_logger) << "Added representation " << representation->getFullName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << "Cannot add duplicate representation " << representation->getFullName();
	}
	return result;
}

bool Manager::addView(std::shared_ptr<View> view)
{
	bool result = false;
	if (std::find(m_views.begin(), m_views.end(), view) == m_views.end())
	{
		m_views.push_back(view);
		SURGSIM_LOG_INFO(m_logger) << "Added view " << view->getFullName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << "Cannot add duplicate view " << view->getFullName();
	}
	return result;
}

bool Manager::removeRepresentation(std::shared_ptr<Representation> representation)
{
	bool result = false;

	auto groupReferences = representation->getGroupReferences();
	for (auto it = groupReferences.cbegin(); it != groupReferences.cend(); ++it)
	{
		m_groups[*it]->remove(representation);
	}

	auto it = std::find(m_representations.begin(), m_representations.end(), representation);
	if (it != m_representations.end())
	{
		m_representations.erase(it);
		SURGSIM_LOG_INFO(m_logger) << "Removed representation " << representation->getFullName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << "Representation not found for removal, " << representation->getFullName();
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
		SURGSIM_LOG_INFO(m_logger) << "Removed view " << view->getFullName();
		result = true;
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << "View not found for removal, " << view->getFullName();
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
	processBehaviors(dt);

	processComponents();

	for (auto& view : m_views)
	{
		view->update(dt);
	}

	for (auto& representation : m_representations)
	{
		representation->update(dt);
	}

	return true;
}

int Manager::getType() const
{
	return SurgSim::Framework::MANAGER_TYPE_GRAPHICS;
}

void SurgSim::Graphics::Manager::addGroup(std::shared_ptr<Group> group)
{
	auto oldGroup = m_groups.find(group->getName());
	SURGSIM_ASSERT(oldGroup == m_groups.end()) <<
		"Tried to add a group that has already been added, " << group->getName();
	m_groups[group->getName()] = group;
	SURGSIM_LOG_INFO(m_logger) << "Added group " << group->getName();
}

void Manager::doBeforeStop()
{
	retireComponents(m_views);
	retireComponents(m_representations);

	ComponentManager::doBeforeStop();
}

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

#include <SurgSim/Graphics/OsgManager.h>

#include <SurgSim/Framework/Log.h>
#include <SurgSim/Graphics/OsgRepresentation.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgView.h>

#include <osgViewer/Scene>
#include <osgDB/WriteFile>

using SurgSim::Graphics::OsgRepresentation;
using SurgSim::Graphics::OsgCamera;
using SurgSim::Graphics::OsgGroup;
using SurgSim::Graphics::OsgManager;

OsgManager::OsgManager() : SurgSim::Graphics::Manager(),
	m_viewer(new osgViewer::CompositeViewer()),
	m_defaultGroup(std::make_shared<OsgGroup>("Default Group"))
{
	m_defaultCamera = std::make_shared<OsgCamera>("Default Camera");
	m_defaultCamera->setGroup(m_defaultGroup);
	m_defaultGroup->getOsgGroup()->getOrCreateStateSet()->setGlobalDefaults();
}

OsgManager::~OsgManager()
{
}

bool OsgManager::setDefaultCamera(std::shared_ptr<OsgCamera> camera)
{
	m_defaultCamera = camera;
	return true;
}
std::shared_ptr<OsgCamera> OsgManager::getDefaultCamera() const
{
	return m_defaultCamera;
}
bool OsgManager::setDefaultGroup(std::shared_ptr<OsgGroup> group)
{
	m_defaultGroup = group;
	return true;
}
std::shared_ptr<OsgGroup> OsgManager::getDefaultGroup() const
{
	return m_defaultGroup;
}

bool OsgManager::addRepresentation(std::shared_ptr<SurgSim::Graphics::Representation> representation)
{
	std::shared_ptr<OsgRepresentation> osgRepresentation = std::dynamic_pointer_cast<OsgRepresentation>(representation);
	if (osgRepresentation && Manager::addRepresentation(osgRepresentation))
	{
		SURGSIM_ASSERT(m_defaultGroup->add(osgRepresentation)) << "Failed to add representation to default group!";

		// Add the component to all the groups that it wants to be in
		std::vector<std::string> requestedGroups = representation->getGroupReferences();
		std::vector<std::shared_ptr<Group>> groups = getGroups();
		for (auto it = std::begin(requestedGroups); it != std::end(requestedGroups); ++it)
		{
			auto groupIt = std::find_if(
				std::begin(groups),
				std::end(groups),
				[it](std::shared_ptr<Group> group){return *it == group->getName();});
			if (groupIt != std::end(groups))
			{
				(*groupIt)->add(representation);
			}
			else
			{
				SURGSIM_LOG_WARNING(getLogger()) << "OsgManager::addRepresentation: " <<
					"The component <" << representation->getName() << "> requested a group <" << *it << "> that could"<<
					" not be found";
			}
		}

		return true;
	}
	else
	{
		SURGSIM_LOG_INFO(getLogger()) << __FUNCTION__ << " Representation is not a subclass of OsgRepresentation " <<
			representation->getName();
		return false;
	}
}
bool OsgManager::addGroup(std::shared_ptr<SurgSim::Graphics::Group> group)
{
	std::shared_ptr<OsgGroup> osgGroup = std::dynamic_pointer_cast<OsgGroup>(group);
	if (osgGroup && Manager::addGroup(osgGroup))
	{
		// Check if there are any representations that might want to be included
		// in this group
		std::string name = group->getName();
		auto representations = getRepresentations();
		for (auto it = std::begin(representations); it != std::end(representations); ++it)
		{
			auto requested = (*it)->getGroupReferences();
			if (std::find(std::begin(requested), std::end(requested), name) != std::end(requested))
			{
				group->add(*it);
			}
		}

		return true;
	}
	else
	{
		SURGSIM_LOG_INFO(getLogger()) << __FUNCTION__ << " Group is not a subclass of OsgGroup " << group->getName();
		return false;
	}
}
bool OsgManager::addView(std::shared_ptr<SurgSim::Graphics::View> view)
{
	std::shared_ptr<OsgView> osgView = std::dynamic_pointer_cast<OsgView>(view);
	if (osgView && Manager::addView(osgView))
	{
		if (! osgView->getCamera())
		{
			osgView->setCamera(m_defaultCamera);
		}
		else if (! osgView->getCamera()->getGroup())
		{
			osgView->getCamera()->setGroup(m_defaultGroup);
		}
		m_viewer->addView(osgView->getOsgView());
		return true;
	}
	else
	{
		SURGSIM_LOG_INFO(getLogger()) << __FUNCTION__ << " View is not a subclass of OsgView " << view->getName();
		return false;
	}
}

bool OsgManager::removeView(std::shared_ptr<SurgSim::Graphics::View> view)
{
	std::shared_ptr<OsgView> osgView = std::dynamic_pointer_cast<OsgView>(view);
	if (osgView)
	{
		m_viewer->removeView(osgView->getOsgView());
	}

	return Manager::removeView(view);
}

bool OsgManager::doInitialize()
{
	return true;
}

bool OsgManager::doStartUp()
{
	return true;
}

bool OsgManager::doUpdate(double dt)
{

	m_defaultCamera->update(dt);

	if (Manager::doUpdate(dt))
	{
		m_viewer->frame();
		return true;
	}
	else
	{
		return false;
	}
}

void OsgManager::doBeforeStop()
{

	// Delete the viewer so that the graphics context will be released in the manager's thread
	m_viewer = nullptr;
}

void SurgSim::Graphics::OsgManager::dumpDebugInfo() const
{
	osgDB::writeNodeFile(*(m_defaultCamera->getOsgCamera()),"default_camera.osgt" );
}


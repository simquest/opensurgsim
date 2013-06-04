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
#include <SurgSim/Graphics/OsgActor.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgView.h>

using SurgSim::Graphics::OsgActor;
using SurgSim::Graphics::OsgCamera;
using SurgSim::Graphics::OsgGroup;
using SurgSim::Graphics::OsgManager;

OsgManager::OsgManager() : SurgSim::Graphics::Manager(),
	m_viewer(new osgViewer::CompositeViewer()),
	m_defaultGroup(std::make_shared<OsgGroup>("Default Group"))
{
	m_defaultCamera = std::make_shared<OsgCamera>("Default Camera");
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

bool OsgManager::addActor(std::shared_ptr<SurgSim::Graphics::Actor> actor)
{
	std::shared_ptr<OsgActor> osgActor = std::dynamic_pointer_cast<OsgActor>(actor);
	if (osgActor && Manager::addActor(osgActor))
	{
		SURGSIM_ASSERT(m_defaultGroup->add(osgActor)) << "Failed to add actor to default group!";
		return true;
	}
	else
	{
		SURGSIM_LOG_INFO(getLogger()) << __FUNCTION__ << " Actor is not a subclass of OsgActor " << actor->getName();
		return false;
	}
}
bool OsgManager::addGroup(std::shared_ptr<SurgSim::Graphics::Group> group)
{
	std::shared_ptr<OsgGroup> osgGroup = std::dynamic_pointer_cast<OsgGroup>(group);
	if (osgGroup && Manager::addGroup(osgGroup))
	{
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
	return addActor(m_defaultCamera) && addGroup(m_defaultGroup);
}

bool OsgManager::doStartUp()
{
	return true;
}

bool OsgManager::doUpdate(double dt)
{
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
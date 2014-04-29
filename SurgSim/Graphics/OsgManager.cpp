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

#include "SurgSim/Graphics/OsgManager.h"

#include <vector>

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/Runtime.h"

#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgGroup.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgScreenSpacePass.h"

#include <osgViewer/Scene>
#include <osgDB/WriteFile>

using SurgSim::Graphics::OsgRepresentation;
using SurgSim::Graphics::OsgCamera;
using SurgSim::Graphics::OsgGroup;
using SurgSim::Graphics::OsgManager;

namespace SurgSim
{
namespace Graphics
{
OsgManager::OsgManager() : SurgSim::Graphics::Manager(),
	m_viewer(new osgViewer::CompositeViewer())
{
}

OsgManager::~OsgManager()
{
}

void addCamera(std::shared_ptr<OsgCamera> newCamera)
{


}

std::shared_ptr<Group> OsgManager::getOrCreateGroup(const std::string& name)
{
	std::shared_ptr<OsgGroup> result;
	auto groups = getGroups();

	auto group = groups.find(name);

	if (group == std::end(groups))
	{
		auto newGroup = std::make_shared<OsgGroup>(name);
		addGroup(newGroup);
		result = newGroup;
	}
	else
	{
		result = std::dynamic_pointer_cast<OsgGroup>(group->second);
	}

	return result;
}

bool OsgManager::addRepresentation(std::shared_ptr<SurgSim::Graphics::Representation> representation)
{
	std::shared_ptr<OsgRepresentation> osgRepresentation = std::dynamic_pointer_cast<OsgRepresentation>(representation);
	if (osgRepresentation && Manager::addRepresentation(osgRepresentation))
	{
		auto camera = std::dynamic_pointer_cast<OsgCamera>(representation);

		// If we have a camera we need to find, and set the group that is is rendering
		if (camera != nullptr)
		{
			camera->setGroup(getOrCreateGroup(camera->getRenderGroupReference()));
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

	bool result = true;
	if (osgView == nullptr || !Manager::addView(view))
	{
		SURGSIM_LOG_WARNING(getLogger()) << __FUNCTION__ << " View is not a subclass of OsgView " << view->getName();
		result = false;
	}
	else
	{
		SURGSIM_ASSERT(view->getCamera() != nullptr) << "View should have a camera when added to the manager.";
		m_viewer->addView(osgView->getOsgView());
	}
	return result;
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
	m_hudElement = std::make_shared<OsgScreenSpacePass>(Representation::DefaultHudGroupName);
	m_hudElement->getCamera()->setGroupReference(Representation::DefaultGroupName);
	return true;
}

bool OsgManager::doStartUp()
{
	return true;
}

bool OsgManager::doUpdate(double dt)
{

	// There is a bug in the scene initialisation where addSceneElement() will not be correctly executed if
	// performed inside of doInitialize(), this needs to be fixed
	// HS-2014-dec-12
	// #workaround
	if (!m_hudElement->isInitialized())
	{
		getRuntime()->getScene()->addSceneElement(m_hudElement);
	}


	if (Manager::doUpdate(dt))
	{
		m_viewer->frame();
		int width;
		int height;

		// \note HS-2013-dec-12 This will work as long as we deal with one view, when we move to stereoscopic
		//	     we might have to revise things. Or just assume that most views have the same size
		getViews()[0]->getDimensions(&width, &height);
		m_hudElement->setViewPort(width, height);
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
	osgDB::writeNodeFile(*m_viewer->getView(0)->getCamera(), "viewer_zero_camera.osgt");
}

}
}



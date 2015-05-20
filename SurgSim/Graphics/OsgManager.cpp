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

void OsgManager::setMultiThreading(bool val)
{
	if (val == true)
	{
		m_viewer->setThreadingModel(osgViewer::ViewerBase::ThreadPerContext);
	}
	else
	{
		m_viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
	}
}

bool OsgManager::isMultiThreading() const
{
	return m_viewer->getThreadingModel() != osgViewer::ViewerBase::SingleThreaded;
}

std::shared_ptr<Group> OsgManager::getOrCreateGroup(const std::string& name)
{
	std::shared_ptr<Group> result;
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
		result = group->second;
	}

	return result;
}

bool OsgManager::addRepresentation(std::shared_ptr<SurgSim::Graphics::Representation> representation)
{
	std::shared_ptr<OsgRepresentation> osgRepresentation = std::dynamic_pointer_cast<OsgRepresentation>(representation);
	bool result;
	if (osgRepresentation)
	{
		result = Manager::addRepresentation(osgRepresentation);
	}
	else
	{
		SURGSIM_LOG_INFO(getLogger())
				<< __FUNCTION__ << " Representation is not a subclass of OsgRepresentation "
				<< representation->getName();
		result = false;
	}
	return result;
}

bool OsgManager::addView(std::shared_ptr<SurgSim::Graphics::View> view)
{
	std::shared_ptr<OsgView> osgView = std::dynamic_pointer_cast<OsgView>(view);

	bool result = true;
	if (osgView == nullptr)
	{
		SURGSIM_LOG_WARNING(getLogger()) << __FUNCTION__ << " View is not a subclass of OsgView " << view->getName();
		result = false;
	}
	else
	{
		SURGSIM_ASSERT(view->getCamera() != nullptr) << "View should have a camera when added to the manager.";
		if (Manager::addView(view))
		{
			m_viewer->addView(osgView->getOsgView());
		}
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

		// \note HS-2013-dec-12 This will work as long as we deal with one view, when we move to stereoscopic
		//	     we might have to revise things. Or just assume that most views have the same size
		if (m_viewer->getNumViews() > 0)
		{
			auto dimensions = getViews()[0]->getDimensions();
			m_hudElement->setViewPort(dimensions[0], dimensions[1]);
		}
		return true;
	}
	else
	{
		return false;
	}
}

void OsgManager::doBeforeStop()
{
#ifdef OSS_DEBUG
	dumpDebugInfo();
#endif
	// Delete the viewer so that the graphics context will be released in the manager's thread
	m_viewer = nullptr;
}

osg::ref_ptr<osgViewer::CompositeViewer> OsgManager::getOsgCompositeViewer() const
{
	return m_viewer;
}

void SurgSim::Graphics::OsgManager::dumpDebugInfo() const
{
	osgDB::writeNodeFile(*m_viewer->getView(0)->getCamera(), "viewer_zero_camera.osgt");
}

}
}



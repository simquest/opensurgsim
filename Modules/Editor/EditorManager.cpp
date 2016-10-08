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

#include "EditorManager.h"

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
#include <osg/NodeVisitor>
#include <osg/NodeCallback>
#include <osg/Matrixf>
#include <osg/Uniform>

using SurgSim::Graphics::OsgRepresentation;
using SurgSim::Graphics::OsgCamera;
using SurgSim::Graphics::OsgGroup;
using SurgSim::Graphics::EditorManager;

namespace
{

/// Class to update the "modelMatrix" uniform for all transforms in the scenegraph
/// #performance This could be change to use a stack of matrices rather than query
/// the nodepath for every transform
class TransformUpdater : public osg::NodeCallback
{
public:
	explicit TransformUpdater(osg::Uniform* uniform) : m_uniform(uniform)
	{

	}

	void operator()(osg::Node* node, osg::NodeVisitor* nodeVisitor) override
	{
		osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nodeVisitor);
		if (cv != nullptr)
		{
			auto mat = osg::computeLocalToWorld(nodeVisitor->getNodePath(), true);
			m_uniform->set(mat);
		}
		traverse(node, nodeVisitor);
	}

private:
	osg::ref_ptr<osg::Uniform> m_uniform;
};

/// Class to find all transform nodes in the added scenegraph, and add the "modelMatrix" uniform
/// to the stateset, also ads the appropriate callback to the node
class TransformModifier : public osg::NodeVisitor
{
public:
	TransformModifier() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{

	}

	void apply(osg::Transform& node) override
	{
		auto state = node.getOrCreateStateSet();
		auto uniform = new osg::Uniform;
		uniform->setName("modelMatrix");
		uniform->setType(osg::Uniform::FLOAT_MAT4);

		osg::Matrix matrix;
		uniform->set(matrix);

		state->addUniform(uniform);

		auto callback = new TransformUpdater(uniform);
		node.addCullCallback(callback);
	}
};

}

namespace SurgSim
{
namespace Graphics
{
EditorManager::EditorManager() : SurgSim::Graphics::Manager(),
	m_viewer(new osgViewer::CompositeViewer())
{
	setMultiThreading(true);
}

EditorManager::~EditorManager()
{
}

void EditorManager::setMultiThreading(bool val)
{
	if (val == true)
	{
		m_viewer->setThreadingModel(osgViewer::ViewerBase::DrawThreadPerContext);
	}
	else
	{
		m_viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
	}
}

bool EditorManager::isMultiThreading() const
{
	return m_viewer->getThreadingModel() != osgViewer::ViewerBase::SingleThreaded;
}

std::shared_ptr<Group> EditorManager::getOrCreateGroup(const std::string& name)
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

bool EditorManager::addRepresentation(std::shared_ptr<SurgSim::Graphics::Representation> representation)
{
	std::shared_ptr<OsgRepresentation> osgRepresentation = std::dynamic_pointer_cast<OsgRepresentation>(representation);
	bool result;
	if (osgRepresentation)
	{
		result = Manager::addRepresentation(osgRepresentation);
		if (result)
		{
			TransformModifier modifier;
			osgRepresentation->getOsgNode()->accept(modifier);
		}
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

bool EditorManager::addView(std::shared_ptr<SurgSim::Graphics::View> view)
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

bool EditorManager::removeView(std::shared_ptr<SurgSim::Graphics::View> view)
{
	std::shared_ptr<OsgView> osgView = std::dynamic_pointer_cast<OsgView>(view);
	if (osgView)
	{
		m_viewer->removeView(osgView->getOsgView());
	}

	return Manager::removeView(view);
}


bool EditorManager::doInitialize()
{
	m_hudElement = std::make_shared<OsgScreenSpacePass>(Representation::DefaultHudGroupName);
	return true;
}

bool EditorManager::doStartUp()
{
	return true;
}

bool EditorManager::doUpdate(double dt)
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

void EditorManager::doBeforeStop()
{
#ifdef OSS_DEBUG
	dumpDebugInfo();
#endif
	// Delete the viewer so that the graphics context will be released in the manager's thread
	m_viewer = nullptr;

	Graphics::Manager::doBeforeStop();
}

osg::ref_ptr<osgViewer::CompositeViewer> EditorManager::getOsgCompositeViewer() const
{
	return m_viewer;
}

void SurgSim::Graphics::EditorManager::dumpDebugInfo() const
{
	osgDB::writeNodeFile(*m_viewer->getView(0)->getCamera(), "viewer_zero_camera.osgt");
}

}
}



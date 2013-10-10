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

#include <SurgSim/Graphics/OsgViewElement.h>

#include <SurgSim/Graphics/OsgView.h>
#include <SurgSim/Graphics/OsgTrackballZoomManipulator.h>

using SurgSim::Graphics::OsgView;
using SurgSim::Graphics::OsgViewElement;

OsgViewElement::OsgViewElement(const std::string& name) :
	SurgSim::Graphics::ViewElement(name, std::make_shared<OsgView>(name + " View"))
{
}
OsgViewElement::~OsgViewElement()
{
}

bool OsgViewElement::setView(std::shared_ptr<SurgSim::Graphics::View> view)
{
	std::shared_ptr<OsgView> osgView = std::dynamic_pointer_cast<OsgView>(view);
	if (osgView && ViewElement::setView(view))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void SurgSim::Graphics::OsgViewElement::enableManipulator(bool val)
{
	if (m_manipulator == nullptr)
	{
		m_manipulator = new OsgTrackballZoomManipulator();
	}

	std::shared_ptr<OsgView> view = std::dynamic_pointer_cast<OsgView>(getView());
	if (view != nullptr)
	{
		if (val)
		{
			view->getOsgView()->setCameraManipulator(m_manipulator);

			// Set a default position
			m_manipulator->setTransformation(osg::Vec3d(2.0f,2.0f,0.0f),
											 osg::Vec3d(0.0f,0.0f,0.0f),
											 osg::Vec3d(0.0f,1.0f,0.0f));
		}
		else
		{
			view->getOsgView()->setCameraManipulator(nullptr);
		}
	}
}

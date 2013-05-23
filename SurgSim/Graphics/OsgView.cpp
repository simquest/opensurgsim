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

#include "SurgSim/Graphics/OsgView.h"

#include <SurgSim/Graphics/OsgCamera.h>

using SurgSim::Graphics::Camera;
using SurgSim::Graphics::OsgCamera;
using SurgSim::Graphics::OsgView;

OsgView::OsgView(const std::string& name) : View(name),
	m_view(new osgViewer::View()),
	m_x(0), m_y(0),
	m_width(800), m_height(600),
	m_isPositionDirty(false),
	m_areDimensionsDirty(false)
{
	/// Don't allow the default camera here, let that be handled at a higher level.
	m_view->setCamera(nullptr);
}

bool OsgView::setPosition(int x, int y)
{
	if (x != m_x || y != m_y)
	{
		m_isPositionDirty = true;
	}
	m_x = x;
	m_y = y;
	return true;
}

void OsgView::getPosition(int* x, int* y)
{
	*x = m_x;
	*y = m_y;
}

bool OsgView::setDimensions(int width, int height)
{
	if (width != m_width || height != m_height)
	{
		m_areDimensionsDirty = true;
	}
	m_width = width;
	m_height = height;
	return true;
}

void OsgView::getDimensions(int* width, int* height)
{
	*width = m_width;
	*height = m_height;
}

bool OsgView::setCamera(std::shared_ptr<Camera> camera)
{
	std::shared_ptr<OsgCamera> osgCamera = std::dynamic_pointer_cast<OsgCamera>(camera);
	if (osgCamera != nullptr && View::setCamera(camera))
	{
		m_view->setCamera(osgCamera->getOsgCamera());
		return true;
	}
	else
	{
		return false;
	}
}

void OsgView::update(double dt)
{
	if (m_isPositionDirty || m_areDimensionsDirty)
	{
		osg::Camera* viewCamera = m_view->getCamera();
		if (viewCamera)
		{
			osgViewer::GraphicsWindow* window = dynamic_cast<osgViewer::GraphicsWindow*>(viewCamera->getGraphicsContext());
			if (window)
			{
				window->setWindowRectangle(m_x, m_y, m_width, m_height);
			}
		}
	}
}

bool OsgView::doInitialize()
{
	m_view->setUpViewInWindow(m_x, m_y, m_width, m_height);
	return true;
}

bool OsgView::doWakeUp()
{
	return true;
}
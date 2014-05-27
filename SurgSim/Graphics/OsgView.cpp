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

#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h"

#include <osgViewer/ViewerEventHandlers>


using SurgSim::Graphics::OsgCamera;
using SurgSim::Graphics::OsgView;

OsgView::OsgView(const std::string& name) : View(name),
	m_x(0), m_y(0),
	m_width(800), m_height(600),
	m_isWindowBorderEnabled(true),
	m_isFirstUpdate(true),
	m_areWindowSettingsDirty(false),
	m_view(new osgViewer::View()),
	m_osgMapUniforms(false)
{
	/// Don't allow the default camera here, let that be handled at a higher level.
	m_view->setCamera(nullptr);
}

bool OsgView::setPosition(int x, int y)
{
	if (x != m_x || y != m_y)
	{
		m_areWindowSettingsDirty = true;
	}
	m_x = x;
	m_y = y;
	return true;
}

void OsgView::getPosition(int* x, int* y) const
{
	*x = m_x;
	*y = m_y;
}

bool OsgView::setDimensions(int width, int height)
{
	if (width != m_width || height != m_height)
	{
		m_areWindowSettingsDirty = true;
	}
	m_width = width;
	m_height = height;
	return true;
}

void OsgView::getDimensions(int* width, int* height) const
{
	*width = m_width;
	*height = m_height;
}

void OsgView::setWindowBorderEnabled(bool enabled)
{
	m_isWindowBorderEnabled = enabled;
	m_areWindowSettingsDirty = true;
}

bool OsgView::isWindowBorderEnabled() const
{
	return m_isWindowBorderEnabled;
}

bool OsgView::setCamera(std::shared_ptr<SurgSim::Graphics::Camera> camera)
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
	if (m_areWindowSettingsDirty)
	{
		osg::Camera* viewCamera = m_view->getCamera();
		if (viewCamera)
		{
			osgViewer::GraphicsWindow* window =
				dynamic_cast<osgViewer::GraphicsWindow*>(viewCamera->getGraphicsContext());
			if (window)
			{
				window->setWindowDecoration(m_isWindowBorderEnabled);
				window->setWindowRectangle(m_x, m_y, m_width, m_height);
				m_areWindowSettingsDirty = false;
			}
		}
	}
}

bool OsgView::doInitialize()
{
	return true;
}

bool OsgView::doWakeUp()
{
	m_view->setUpViewInWindow(m_x, m_y, m_width, m_height);

	auto statsHandler = new osgViewer::StatsHandler;
	m_view->addEventHandler(statsHandler);

	if (m_osgMapUniforms)
	{
		fixupStatsHandler(statsHandler);

		m_view->getCamera()->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);
		m_view->getCamera()->getGraphicsContext()->getState()->setUseVertexAttributeAliasing(true);
	}

	return true;
}

void OsgView::fixupStatsHandler(osgViewer::StatsHandler* statsHandler)
{
	// use ref_ptr in case loading fails we don't have to clean up
	osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX);
	osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT);

	bool success = true;
	std::string fileName;
	if (getRuntime()->getApplicationData()->tryFindFile("Shaders/osg_statshandler.vert", &fileName))
	{
		success = vertexShader->loadShaderSourceFromFile(fileName);
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Could not find Shaders/osg_statshandler.vert, the osg stats "
				<< "display will probably not work correctly.";
		success = false;
	}

	if (getRuntime()->getApplicationData()->tryFindFile("Shaders/osg_statshandler.frag", &fileName))
	{
		success = fragmentShader->loadShaderSourceFromFile(fileName);
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Could not find Shaders/osg_statshandler.frag, the osg stats "
				<< "display will probably not work correctly.";
		success = false;
	}

	if (success)
	{
		osg::ref_ptr<osg::Program> program = new osg::Program;
		program->addShader(fragmentShader);
		program->addShader(vertexShader);

		auto state = statsHandler->getCamera()->getOrCreateStateSet();

		auto texture = new osg::Texture2D();
		texture->setTextureSize(256, 256);

		state->setAttributeAndModes(program);
		state->setTextureAttributeAndModes(0, texture);
		state->addUniform(new osg::Uniform("osg_TextTexture", (int) 0));
	}
}

void SurgSim::Graphics::OsgView::setOsgMapsUniforms(bool val)
{
	SURGSIM_ASSERT(!isInitialized()) << "Can't change mapping mode after initialization.";
	m_osgMapUniforms = val;
}

bool SurgSim::Graphics::OsgView::getOsgMapsUniforms()
{
	return m_osgMapUniforms;
}

osg::ref_ptr<osgViewer::View> SurgSim::Graphics::OsgView::getOsgView() const
{
	return m_view;
}


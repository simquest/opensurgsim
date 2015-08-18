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

#include "SurgSim/Graphics/OsgScreenSpacePass.h"
#include "SurgSim/Graphics/OsgCamera.h"

#include <memory>
#include <osg/Camera>

namespace SurgSim
{
namespace Graphics
{

OsgScreenSpacePass::OsgScreenSpacePass(const std::string& name) :
	RenderPass(name),
	m_width(0),
	m_height(0)
{

}

OsgScreenSpacePass::~OsgScreenSpacePass()
{

}

bool OsgScreenSpacePass::doInitialize()
{
	RenderPass::doInitialize();
	auto camera = std::dynamic_pointer_cast<OsgCamera>(getCamera());

	bool result = false;

	if (camera != nullptr)
	{
		m_camera = camera->getOsgCamera();
		m_camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
		m_camera->setClearMask(0x0);
		setRenderOrder(Camera::RENDER_ORDER_POST_RENDER, 0);
		updateViewport(m_width, m_height);
		result = true;
	}

	return result;
}


void OsgScreenSpacePass::updateViewport(int width, int height)
{
	if (m_camera != nullptr)
	{
		m_camera->setProjectionMatrixAsOrtho2D(0, width, 0, height);
	}
}

void OsgScreenSpacePass::setViewPort(int width, int height)
{
	SURGSIM_ASSERT(width >= 0 && height >= 0) << "Viewport width or height cannot be smaller than 0.";

	if (m_width != width || m_height != height)
	{
		m_width = width;
		m_height = height;

		updateViewport(m_width, m_height);
	}
}


}
}


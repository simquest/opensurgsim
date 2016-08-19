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

#include "SurgSim/Graphics/RenderPass.h"

#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgGroup.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h"
#include "SurgSim/Graphics/View.h"
#include "SurgSim/Graphics/Texture.h"

namespace SurgSim
{
namespace Graphics
{

RenderPass::RenderPass(const std::string& name) :
	SceneElement(name)
{
	m_camera = std::make_shared<OsgCamera>(getName() + " camera");
	m_camera->setRenderGroupReference(name);

	m_material = std::make_shared<OsgMaterial>("material");
	m_camera->setMaterial(m_material);
}

RenderPass::~RenderPass()
{

}

bool RenderPass::doInitialize()
{
	addComponent(m_camera);
	addComponent(m_material);
	return true;
}

std::shared_ptr<Camera> RenderPass::getCamera()
{
	return m_camera;
}

bool RenderPass::setRenderTarget(std::shared_ptr<RenderTarget> target)
{
	bool result = m_camera->setRenderTarget(target);
	if (result)
	{
		m_renderTarget = target;
	}
	return result;
}

std::shared_ptr<RenderTarget> RenderPass::getRenderTarget()
{
	return m_renderTarget;
}

bool RenderPass::setMaterial(std::shared_ptr<SurgSim::Framework::Component> material)
{
	bool result = m_camera->setMaterial(material);
	if (result)
	{
		m_material = std::dynamic_pointer_cast<Material>(material);
	}
	return result;
}

std::shared_ptr<Material> RenderPass::getMaterial()
{
	return m_material;
}

void RenderPass::setRenderOrder(SurgSim::Graphics::Camera::RenderOrder order, int value)
{
	m_camera->setRenderOrder(order, value);
}

void RenderPass::showColorTarget(int x, int y, int width, int height)
{
	if (m_debugColor == nullptr && m_renderTarget->getColorTargetCount() > 0)
	{
		auto texture = m_renderTarget->getColorTarget(0);
		m_debugColor = buildDebugQuad("debug color", texture);
	}
	if (m_debugColor != nullptr)
	{
		m_debugColor->setLocation(x, y);
		m_debugColor->setSize(width, height);
		m_debugColor->setLocalActive(true);
	}
}

void RenderPass::hideColorTarget()
{
	if (m_debugColor != nullptr)
	{
		m_debugColor->setLocalActive(false);
	}
}

void RenderPass::showDepthTarget(int x, int y, int width, int height)
{
	if (m_debugDepth == nullptr && m_renderTarget->doesUseDepthTarget())
	{
		auto texture = m_renderTarget->getDepthTarget();
		m_debugDepth = buildDebugQuad("debug depth", texture);
	}
	if (m_debugDepth != nullptr)
	{
		m_debugDepth->setLocation(x, y);
		m_debugDepth->setSize(width, height);
		m_debugDepth->setLocalActive(true);
	}
}

void RenderPass::hideDepthTarget()
{
	if (m_debugDepth != nullptr)
	{
		m_debugDepth->setLocalActive(false);
	}
}

std::shared_ptr<ScreenSpaceQuadRepresentation> RenderPass::buildDebugQuad(const std::string& name,
																		  std::shared_ptr<Texture> texture)
{
	auto result = std::make_shared<OsgScreenSpaceQuadRepresentation>(name);
	result->setTexture(texture);
	addComponent(result);
	return result;
}


}; // Graphics
}; // SurgSim

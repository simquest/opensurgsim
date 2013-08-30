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

#include <SurgSim/Framework/Log.h>
#include <SurgSim/Graphics/OsgRenderPass.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgRenderTarget.h>

using SurgSim::Framework::Logger;

namespace SurgSim
{
namespace Graphics
{

OsgRenderPass::OsgRenderPass(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	RenderPass(name),
	m_camera(std::make_shared<OsgCamera>(name + " Default Camera")),
	m_group(std::make_shared<OsgGroup>(name + " Default Group"))
{
	m_camera->setGroup(m_group);
}

OsgRenderPass::~OsgRenderPass()
{

}

std::shared_ptr<Camera> OsgRenderPass::getCamera() const
{
	return m_camera;
}

bool OsgRenderPass::setCamera(std::shared_ptr<Camera> val)
{
	bool result = false;
	std::shared_ptr<OsgCamera> camera = std::dynamic_pointer_cast<OsgCamera>(val);
	if (camera != nullptr)
	{
		m_camera = camera;
		result = m_camera->setGroup(m_group);
	}
	else
	{
		SURGSIM_LOG_WARNING(Logger::getLogger("Graphics/OsgRenderPass")) << 
			"Could not assign camera to renderpass the camera not the correct implementation type.";	
	}
	return result;
}

std::shared_ptr<Group> OsgRenderPass::getGroup() const
{
	return m_group;
}

bool OsgRenderPass::setGroup(std::shared_ptr<Group> val)
{
	bool result = false;
	std::shared_ptr<OsgGroup> group = std::dynamic_pointer_cast<OsgGroup>(val);
	if (group != nullptr)
	{
		m_group = group;
		result = m_camera->setGroup(m_group);
	}
	else
	{
		SURGSIM_LOG_WARNING(Logger::getLogger("Graphics/OsgRenderPass")) << 
			"Could not assign group to renderpass, the group is not the correct implementation type.";
	}
	return result;
}

std::shared_ptr<Material> OsgRenderPass::getMaterial() const
{
	return m_material;
}

bool OsgRenderPass::setMaterial(std::shared_ptr<Material> val)
{
	bool result = false;
	std::shared_ptr<OsgMaterial> material = std::dynamic_pointer_cast<OsgMaterial>(val);
	if (material != nullptr)
	{
		m_material = material;
		m_camera->setMaterial(material);
		result = true;
	}
	else
	{
		SURGSIM_LOG_WARNING(Logger::getLogger("Graphics/OsgRenderPass")) << 
			"Could not assign material to renderpass, the material is not the correct implementation type.";
	}
	return result;
}

std::shared_ptr<RenderTarget> OsgRenderPass::getRenderTarget() const
{
	return m_renderTarget;
}

bool OsgRenderPass::setRenderTarget(std::shared_ptr<RenderTarget> val)
{
	bool result = false;
	std::shared_ptr<OsgAbstractRenderTarget> renderTarget = std::dynamic_pointer_cast<OsgAbstractRenderTarget>(val);
	if (renderTarget != nullptr)
	{
		m_renderTarget = renderTarget;
		m_camera->setRenderTarget(m_renderTarget);
		result = true;
	}
	else
	{
		SURGSIM_LOG_WARNING(Logger::getLogger("Graphics/OsgRenderPass")) << 
			"Could not assign RenderTarget to RenderPass, the RenderTarget is not the correct implementation type";	
	}
	return result;
}

osg::ref_ptr<osg::Node> OsgRenderPass::getOsgNode()
{
	return m_camera->getOsgNode();
}

}; // Graphics
}; // SurgSim

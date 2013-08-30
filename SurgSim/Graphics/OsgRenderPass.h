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

#ifndef SURGSIM_GRAPHICS_OSGRENDERPASS_H
#define SURGSIM_GRAPHICS_OSGRENDERPASS_H

#include <memory>
#include <SurgSim/Graphics/RenderPass.h>
#include <SurgSim/Graphics/OsgRepresentation.h>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace SurgSim
{
namespace Graphics
{

class OsgCamera;
class OsgGroup;
class OsgMaterial;
class OsgAbstractRenderTarget;

class Camera;
class Group;
class Material;
class RenderTarget;

class OsgRenderPass : public OsgRepresentation, public RenderPass
{
public:

	/// Constructor
	OsgRenderPass(const std::string& name);
	~OsgRenderPass();

	virtual bool setCamera(std::shared_ptr<Camera> val) override;
	virtual std::shared_ptr<Camera> getCamera() const override;
	virtual bool setGroup(std::shared_ptr<Group> val) override;
	virtual std::shared_ptr<Group> getGroup() const override;
	virtual bool setMaterial(std::shared_ptr<Material> val) override;
	virtual std::shared_ptr<Material> getMaterial() const override;
	virtual bool setRenderTarget(std::shared_ptr<RenderTarget> val) override;
	virtual std::shared_ptr<RenderTarget> getRenderTarget() const override;

	virtual osg::ref_ptr<osg::Node> getOsgNode();

private:

	std::shared_ptr<OsgCamera> m_camera;
	std::shared_ptr<OsgGroup> m_group;
	std::shared_ptr<OsgMaterial> m_material;
	std::shared_ptr<OsgAbstractRenderTarget> m_renderTarget;
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}; // graphics
}; // SurgSim

#endif
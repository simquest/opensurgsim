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

#ifndef SURGSIM_GRAPHICS_RENDERPASS_H
#define SURGSIM_GRAPHICS_RENDERPASS_H

#include <string>
#include <memory>
#include <SurgSim/Graphics/Representation.h>

namespace SurgSim
{
namespace Graphics
{

class Camera;
class Group;
class Material;
class RenderTarget;

class RenderPass : public virtual Representation
{
public:

	/// Constructor
	RenderPass(const std::string name) : Representation(name) 
	{

	}

	~RenderPass()
	{

	}

	virtual bool setCamera(std::shared_ptr<Camera> camera) = 0;
	virtual std::shared_ptr<Camera> getCamera() const = 0;

	virtual bool setGroup(std::shared_ptr<Group> group) = 0;
	virtual std::shared_ptr<Group> getGroup() const = 0;

	virtual bool setRenderTarget(std::shared_ptr<RenderTarget> target) = 0;
	virtual std::shared_ptr<RenderTarget> getRenderTarget() const = 0;

	virtual bool setMaterial(std::shared_ptr<Material> material) = 0;
	virtual std::shared_ptr<Material> getMaterial() const = 0;

};

}; // Graphics
}; // SurgSim

#endif
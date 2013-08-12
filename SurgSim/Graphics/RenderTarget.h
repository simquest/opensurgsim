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

#ifndef SURGSIM_GRAPHICS_RENDERTARGET_H
#define SURGSIM_GRAPHICS_RENDERTARGET_H

#include <memory>

namespace SurgSim
{
namespace Graphics
{

class Texture;

class RenderTarget
{
public:

	/// Constructor
	RenderTarget()
	{
	};
	
	~RenderTarget()
	{
	};

	virtual void setSize(int width, int height) = 0;
	virtual void getSize(int* width, int* height) const = 0;

	virtual void setScale(double scale) = 0;
	virtual double getScale() const = 0;

	/// Sets the number of color target textures to use, returns the actual number of textures
	/// that can be used.
	/// \param	count	Number of textures to use as targets.
	/// \return	number of textures that will be used.
	virtual int setColorTargetCount(int count) = 0;
	virtual int getColorTargetCount() const = 0;
	virtual std::shared_ptr<Texture> getColorTarget(int index) const = 0;

	virtual void useDepthTarget(bool val) = 0;
	virtual bool doesUseDepthTarget() const = 0;
	virtual std::shared_ptr<Texture> getDepthTarget() const = 0;

	virtual void useStencilTarget(bool val) = 0;
	virtual bool doesUseStencilTarget() const = 0;
	virtual std::shared_ptr<Texture> getStencilTarget() const = 0;

private:

};

}; // Graphics
}; // SurgSim

#endif
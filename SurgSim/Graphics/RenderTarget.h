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

/// RenderTarget is an abstraction of the target buffers that a Camera should use to render it's scene
/// valid targets are a given number of color buffers, and an optional depth buffer. The buffers need to be
/// made available as textures so they can be reused in another rendering step. The type of texture is not
/// determined at this point and will depend on the concrete RenderTarget that is instantiated.
/// The RenderTarget is considered immutable after construction.
class RenderTarget
{
public:

	/// Constructor
	RenderTarget()
	{
	};

	virtual ~RenderTarget()
	{
	};

	/// Gets a size.
	/// \param [out]	width, height	The width and height of the RenderTarget textures.
	virtual void getSize(int* width, int* height) const = 0;

	/// Returns the number of textures that this RenderTarget uses to draw into.
	/// \return	The color target count.
	virtual int getColorTargetCount() const = 0;

	/// Gets the indicated texture that is used as a target.
	/// \param	index	Zero-based index of the texture to be used.
	/// \return	The color target, nullptr if index exceeds getColorTargetCount().
	virtual std::shared_ptr<Texture> getColorTarget(int index) const = 0;

	/// Check wether this draws into a depth texture.
	/// \return	true if yes, otherwise false.
	virtual bool doesUseDepthTarget() const = 0;

	/// Returns the texture that is used for the depth map drawing.
	/// \return	The depth target.
	virtual std::shared_ptr<Texture> getDepthTarget() const = 0;

private:

};

}; // Graphics
}; // SurgSim

#endif
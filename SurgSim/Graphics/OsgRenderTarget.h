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

#ifndef SURGSIM_GRAPHICS_OSGRENDERTARGET_H
#define SURGSIM_GRAPHICS_OSGRENDERTARGET_H

#include <memory>
#include <unordered_map>

#include <SurgSim/Graphics/RenderTarget.h>
#include <SurgSim/Graphics/OsgTexture2d.h>
#include <SurgSim/Graphics/OsgTextureRectangle.h>

#include <osg/FrameBufferObject>

namespace SurgSim
{
namespace Graphics
{
/// Speficig implementation of the render target class.
/// \tparam	T	Type of the texture that should be used as targets probably either OsgTexture2d or OsgTextureRectangle.
template <class T>
class OsgRenderTarget : public RenderTarget
{
public:

	typedef T TextureType;

	/// Default constructor
	OsgRenderTarget();

	/// Constructor set all the paramters for the render target
	/// \param	width, height	The width and height of the target textures.
	/// \param	scale	  	(Optional) the scale, scales width and height by this factor.
	/// \param	colorCount	(Optional) number of color textures to use.
	/// \param	useDepth  	(Optional) whether to use a depth texture.
	OsgRenderTarget(int width, int height, double scale = 1.0,
					int colorCount = 0, bool useDepth = false);
	/// Destructor
	~OsgRenderTarget();

	/// Gets a size.
	/// \param [out]	width, height	The width and height of the RenderTarget textures.
	virtual void getSize(int* width, int* height) const override;

	virtual int getColorTargetCount() const override;
	virtual std::shared_ptr<Texture> getColorTarget(int index) const override;
	std::shared_ptr<TextureType> getColorTargetOsg(int index) const;

	virtual bool doesUseDepthTarget() const override;
	virtual std::shared_ptr<Texture> getDepthTarget() const override;
	std::shared_ptr<TextureType> getDepthTargetOsg() const;

private:

	/// Values that represent TargetTypes.
	enum TargetTypes {
		TARGETTYPE_DEPTH = 0,
		TARGETTYPE_COLORBASE = 1
	};

	/// The width of this RenderTarget.
	int m_width;

	/// The height of this RenderTarget.
	int m_height;

	/// Number of color targets.
	int m_colorTargetCount;

	/// The textures that are being used as target, size of this is 16 (ColorTargets) + 1 (Depth).
	std::vector<std::shared_ptr<TextureType>> m_textures;

	/// Sets color target count.
	/// \param	size	The number of color textures to use.
	/// \return	.
	int setColorTargetCount(int count);

	/// Use depth target.
	/// \param	val	true to value.
	void useDepthTarget(bool val);

	/// Sets up the texture with a given target type (depth or color w/ index).
	/// \param	type	The index of the texture to use.
	void setupTexture(int type);
};

#include <SurgSim/Graphics/OsgRenderTarget-inl.h>

typedef OsgRenderTarget<OsgTexture2d> OsgRenderTarget2d;
typedef OsgRenderTarget<OsgTextureRectangle> OsgRenderTargetRectangle;

}; // Graphics
}; // SurgSim

#endif
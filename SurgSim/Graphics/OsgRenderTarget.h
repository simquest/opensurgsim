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

#include "SurgSim/Graphics/RenderTarget.h"
#include "SurgSim/Graphics/OsgTexture.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgTextureRectangle.h"

#include <osg/FrameBufferObject>

namespace SurgSim
{
namespace Graphics
{


/// Osg abstract render target, this hides the type of the actual osg texture and lets us use
/// OsgRenderTarget without the template type
class OsgAbstractRenderTarget : public RenderTarget
{
	/// Accessor for the color target as an OsgTexture.
	/// \param	index	Zero-based index of the color texture.
	/// \return	The color target as an osg specific class.
	virtual std::shared_ptr<OsgTexture> getColorTargetOsg(int index) const = 0;

	/// Accessor for the depth target as an OsgTexture.
	/// \return	The depth target as an osg specific class.
	virtual std::shared_ptr<OsgTexture> getDepthTargetOsg() const = 0;
};

/// Specific implementation of the render target class. It is templated so different texture formats can be chosen.
/// \tparam	T	Type of the texture that should be used as targets probably either OsgTexture2d or OsgTextureRectangle.
template <class T>
class OsgRenderTarget : public OsgAbstractRenderTarget
{
public:

	/// The internal type of the texture, not exposed in the public interface
	typedef T TextureType;

	/// Default constructor
	OsgRenderTarget();

	/// Constructor set all the paramters for the render target
	/// \param	width, height	The width and height of the target textures.
	/// \param	scale	  	(Optional) the scale, scales width and height by this factor.
	/// \param	colorCount	(Optional) number of color textures to use.
	/// \param	useDepth  	(Optional) whether to use a depth texture.
	/// \param  useFloat	(Optional) whether to use float color buffers
	OsgRenderTarget(int width, int height, double scale = 1.0,
		int colorCount = 0, bool useDepth = false, bool useFloat = false);
	/// Destructor
	~OsgRenderTarget();

	/// Gets a size.
	/// \param [out]	width, height	The width and height of the RenderTarget textures.
	void getSize(int* width, int* height) const override;

	/// \return	The number of color targets that are available.
	int getColorTargetCount() const override;

	/// Generic accessor for a specific color target texture.
	/// \param	index	Zero-based index of the texture.
	/// \return	The actual Texture.
	std::shared_ptr<Texture> getColorTarget(int index) const override;

	/// Accessor for the color target as an OsgTexture.
	/// \param	index	Zero-based index of the color texture.
	/// \return	The color target as an osg specific class.
	std::shared_ptr<OsgTexture> getColorTargetOsg(int index) const;

	/// Determines if RenderTarget does use a depth target.
	/// \return	true if it does.
	bool doesUseDepthTarget() const override;

	/// Generic accessor for the depth Target.
	/// \return	The depth target.
	std::shared_ptr<Texture> getDepthTarget() const override;

	/// Accessor for the depth target as an OsgTexture.
	/// \return	The depth target as an osg specific class.
	std::shared_ptr<OsgTexture> getDepthTargetOsg() const;

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
	/// \param	count	The number of color textures to use.
	/// \param  floatColor Use float type for color textures 
	/// \return	.
	int setColorTargetCount(int count, bool floatColor);

	/// Use depth target.
	/// \param	val	true to value.
	void useDepthTarget(bool val);

	/// Sets up the texture with a given target type (depth or color w/ index).
	/// \param	type	The index of the texture to use.
	/// \param  floatColor	Use bool for float color textures 
	void setupTexture(int type, bool floatColor);
};

///@{
/// Predefine specialized render targets
typedef OsgRenderTarget<OsgTexture2d> OsgRenderTarget2d;
typedef OsgRenderTarget<OsgTextureRectangle> OsgRenderTargetRectangle;
///@}

}; // Graphics
}; // SurgSim

#include "SurgSim/Graphics/OsgRenderTarget-inl.h"

#endif
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

#ifndef SURGSIM_GRAPHICS_OSGTEXTURERECTANGLE_H
#define SURGSIM_GRAPHICS_OSGTEXTURERECTANGLE_H

#include "SurgSim/Graphics/OsgTexture.h"
#include "SurgSim/Graphics/TextureRectangle.h"

#include <osg/TextureRectangle>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace SurgSim
{

namespace Graphics
{

/// OSG implementation of a Rectangle Texture
///
/// Wraps an osg::TextureRectangle
///
/// \note	Texel coordinates are used to access this texture.
class OsgTextureRectangle : public OsgTexture, public TextureRectangle
{
public:
	/// Constructor
	/// \post	No image is loaded in the texture.
	OsgTextureRectangle();

	/// Sets the size of the texture
	/// \param	width	Width of the texture
	/// \param	height	Height of the texture
	/// \note	Use this to setup a texture as a render target rather than loading from file.
	virtual void setSize(int width, int height);

	/// Gets the size of the texture
	/// \param[out]	width	Width of the texture
	/// \param[out]	height	Height of the texture
	virtual void getSize(int* width, int* height) const;

	/// Returns the osg::TextureRectangle
	osg::ref_ptr<osg::TextureRectangle> getOsgTextureRectangle() const
	{
		return static_cast<osg::TextureRectangle*>(getOsgTexture().get());
	}
};

};  // namespace Graphics

};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGTEXTURERECTANGLE_H

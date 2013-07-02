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

#ifndef SURGSIM_GRAPHICS_OSGTEXTURE1D_H
#define SURGSIM_GRAPHICS_OSGTEXTURE1D_H

#include <SurgSim/Graphics/OsgTexture.h>
#include <SurgSim/Graphics/Texture1d.h>

#include <osg/Texture1D>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace SurgSim
{

namespace Graphics
{

/// OSG implementation of a 1D Texture
///
/// Wraps an osg::Texture1d
class OsgTexture1d : public OsgTexture, public Texture1d
{
public:
	/// Constructor
	/// \post	No image is loaded in the texture.
	OsgTexture1d();

	/// Sets the size of the texture
	/// \param	width	Width of the texture
	/// \note	Use this to setup a texture as a render target rather than loading from file.
	virtual void setSize(int width);

	/// Gets the size of the texture
	/// \return	width	Width of the texture
	virtual void getSize(int* width) const;

	/// Returns the osg::Texture1D
	osg::ref_ptr<osg::Texture1D> getOsgTexture1d() const
	{
		return static_cast<osg::Texture1D*>(getOsgTexture().get());
	}
};

};  // namespace Graphics

};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGTEXTURE1D_H

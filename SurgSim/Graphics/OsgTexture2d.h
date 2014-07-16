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

#ifndef SURGSIM_GRAPHICS_OSGTEXTURE2D_H
#define SURGSIM_GRAPHICS_OSGTEXTURE2D_H

#include "SurgSim/Graphics/OsgTexture.h"
#include "SurgSim/Graphics/Texture2d.h"

#include <osg/Texture2D>

#include "SurgSim/Framework/Accessible.h"
#include <boost/any.hpp>
#include <memory>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace SurgSim
{

namespace Graphics
{

/// OSG implementation of a 2D Texture
///
/// Wraps an osg::Texture2d
class OsgTexture2d : public OsgTexture, public Texture2d
{
public:
	/// Constructor
	/// \post	No image is loaded in the texture.
	OsgTexture2d();

	/// Sets the size of the texture
	/// \param	width	Width of the texture
	/// \param	height	Height of the texture
	/// \note	Use this to setup a texture as a render target rather than loading from file.
	virtual void setSize(int width, int height);

	/// Gets the size of the texture
	/// \param[out]	width	Width of the texture
	/// \param[out]	height	Height of the texture
	virtual void getSize(int* width, int* height) const;

	/// Returns the osg::Texture2D
	osg::ref_ptr<osg::Texture2D> getOsgTexture2d() const
	{
		return static_cast<osg::Texture2D*>(getOsgTexture().get());
	}
};

};  // namespace Graphics


namespace Framework
{
template <>
std::shared_ptr<SurgSim::Graphics::OsgTexture2d> convert(boost::any val);
}

};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGTEXTURE2D_H

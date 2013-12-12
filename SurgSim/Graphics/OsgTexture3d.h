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

#ifndef SURGSIM_GRAPHICS_OSGTEXTURE3D_H
#define SURGSIM_GRAPHICS_OSGTEXTURE3D_H

#include "SurgSim/Graphics/OsgTexture.h"
#include "SurgSim/Graphics/Texture3d.h"

#include <osg/Texture3D>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace SurgSim
{

namespace Graphics
{

/// OSG implementation of a 3D Texture
///
/// Wraps an osg::Texture3d
class OsgTexture3d : public OsgTexture, public Texture3d
{
public:
	/// Constructor
	/// \post	No image is loaded in the texture.
	OsgTexture3d();

	/// Sets the size of the texture
	/// \param	width	Width of the texture
	/// \param	height	Height of the texture
	/// \param	depth	Depth of the texture
	/// \note	Use this to setup a texture as a render target rather than loading from file.
	virtual void setSize(int width, int height, int depth);

	/// Gets the size of the texture
	/// \param[out]	width	Width of the texture
	/// \param[out]	height	Height of the texture
	/// \param[out]	depth	Depth of the texture
	virtual void getSize(int* width, int* height, int* depth) const;

	/// Loads images slices from files into the 3D texture
	/// \param	filePaths	Paths to the image files
	/// \return	True if the image is successfully loaded, otherwise false
	/// \note	The slices are stacked in the order provided to create the depth of the 3D texture.
	virtual bool loadImageSlices(const std::vector<std::string>& filePaths);

	/// Returns the osg::Texture3D
	osg::ref_ptr<osg::Texture3D> getOsgTexture3d() const
	{
		return static_cast<osg::Texture3D*>(getOsgTexture().get());
	}
};

};  // namespace Graphics

};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGTEXTURE3D_H

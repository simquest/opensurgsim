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

#ifndef SURGSIM_GRAPHICS_OSGTEXTURE_H
#define SURGSIM_GRAPHICS_OSGTEXTURE_H

#include <SurgSim/Graphics/Texture.h>

#include <osg/Texture>

namespace SurgSim
{

namespace Graphics
{

/// Base class for OSG implementations of Graphics Textures.
///
/// Wraps an osg::Texture.
class OsgTexture : public virtual Texture
{
public:
	/// Loads an image into the texture from a file
	/// \param	filePath	Path to the image file
	/// \return	True if the image is successfully loaded, otherwise false
	virtual bool loadImage(const std::string& filePath);

	/// Removes the image from the texture
	virtual void clearImage();

	/// Returns the osg::Texture1D
	osg::ref_ptr<osg::Texture> getOsgTexture() const
	{
		return m_texture;
	}

protected:
	/// Constructor
	/// \param	texture	OSG texture
	explicit OsgTexture(osg::Texture* texture);

private:
	/// OSG texture
	osg::ref_ptr<osg::Texture> m_texture;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGTEXTURE_H

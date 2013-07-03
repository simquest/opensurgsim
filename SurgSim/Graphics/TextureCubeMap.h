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

#ifndef SURGSIM_GRAPHICS_TEXTURECUBEMAP_H
#define SURGSIM_GRAPHICS_TEXTURECUBEMAP_H

#include <SurgSim/Graphics/Texture.h>

namespace SurgSim
{

namespace Graphics
{

/// Base class defining the interface for a Cube Map Graphics Texture.
/// A Cube Map Texture has a width and height, which is the same for each face of the cube.
class TextureCubeMap : public virtual Texture
{
public:
	/// Sets the size of the texture, which is the same for each face of the cube
	/// \param	width	Width of the texture
	/// \param	height	Height of the texture
	/// \note	Use this to setup a texture as a render target rather than loading from file.
	virtual void setSize(int width, int height) = 0;

	/// Gets the size of the texture, which is the same for each face of the cube
	/// \param[out]	width	Width of the texture
	/// \param[out]	height	Height of the texture
	virtual void getSize(int* width, int* height) const = 0;

	/// Loads images from files into the faces of the cube map
	/// \param	negativeX	Path to the image for the (-X) face
	/// \param	positiveX	Path to the image for the (+X) face
	/// \param	negativeY	Path to the image for the (-Y) face
	/// \param	positiveY	Path to the image for the (+Y) face
	/// \param	negativeZ	Path to the image for the (-Z) face
	/// \param	positiveZ	Path to the image for the (+Z) face
	/// \return	True if the image is successfully loaded, otherwise false
	virtual bool loadImageFaces(const std::string& negativeX, const std::string& positiveX,
		const std::string& negativeY, const std::string& positiveY,
		const std::string& negativeZ, const std::string& positiveZ) = 0;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_TEXTURECUBEMAP_H

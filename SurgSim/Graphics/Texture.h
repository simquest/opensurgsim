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

#ifndef SURGSIM_GRAPHICS_TEXTURE_H
#define SURGSIM_GRAPHICS_TEXTURE_H

#include <string>

namespace SurgSim
{

namespace Graphics
{

/// Base class defining the interface for a Graphics Texture.
class Texture
{
public:
	/// Destructor
	virtual ~Texture()
	{
	}

	/// Loads an image into the texture from a file
	/// \param	filePath	Path to the image file
	/// \return	True if the image is successfully loaded, otherwise false
	virtual bool loadImage(const std::string& filePath) = 0;

	/// Removes the image from the texture
	virtual void clearImage() = 0;

	void setIsPointSprite(bool value)
	{
		m_isPointSprite = value;
	}

	bool isPointSprite()
	{
		return m_isPointSprite;
	}

private:
	bool m_isPointSprite;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_TEXTURE_H

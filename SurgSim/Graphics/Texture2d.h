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

#ifndef SURGSIM_GRAPHICS_TEXTURE2D_H
#define SURGSIM_GRAPHICS_TEXTURE2D_H

#include "SurgSim/Graphics/Texture.h"

namespace SurgSim
{

namespace Graphics
{

/// Base class defining the interface for a 2D Graphics Texture.
/// A 2D Texture has width and height.
/// \note	Normalized texture coordinates are used to access this texture.
class Texture2d : public virtual Texture
{
public:
	/// Sets the size of the texture
	/// \param	width	Width of the texture
	/// \param	height	Height of the texture
	/// \note	Use this to setup a texture as a render target rather than loading from file.
	virtual void setSize(int width, int height) = 0;

	/// Gets the size of the texture
	/// \param[out]	width	Width of the texture
	/// \param[out]	height	Height of the texture
	virtual void getSize(int* width, int* height) const = 0;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_TEXTURE2D_H

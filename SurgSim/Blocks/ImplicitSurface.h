// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_BLOCKS_IMPLICITSURFACE_H
#define SURGSIM_BLOCKS_IMPLICITSURFACE_H

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Graphics/Uniform.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Blocks
{
static const std::string GROUP_IMPLICIT_SURFACE = "ImplicitSurface";

/// Builds a series of SceneElements enabling the rendering of a screen-space surface, all graphics objects that
/// should be rendered as a surface need to be in the render group GROUP_IMPLICIT_SURFACE.
/// The rest is done by the graphics system.
/// All of the elements added are \sa RenderPass elements
/// \param camera the view camera that is used for this pass
/// \param sphereRadius the radius in meters that each point sprite sphere should have
/// \param sphereScale the scaling factor for the point sprite sphere based on distance from the camera
/// \param textureSize the size of the render textures, not including the final pass that takes the size of the
///			screen
/// \param diffuseColor the color to use for the final surface shading
/// \param specularColor the color to use for the specular hightlight on the surface
/// \param shininess the shiniess factor for the specular highlight
/// \param showDebug whether to show debug information
std::vector<std::shared_ptr<Framework::SceneElement>> createImplicitSurface(std::shared_ptr<Framework::Component> camera,
			std::shared_ptr<Framework::Component> light,
			float sphereRadius,
			float sphereScale,
			int textureSize,
			const Math::Vector4f& diffuseColor,
			const Math::Vector4f& specularColor,
			float shininess,
			bool showDebug);

} // Blocks
} // SurgSim

#endif // SURGSIM_BLOCKS_IMPLICITSURFACE_H

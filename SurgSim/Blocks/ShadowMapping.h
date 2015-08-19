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

#ifndef SURGSIM_BLOCKS_SHADOWMAPPING_H
#define SURGSIM_BLOCKS_SHADOWMAPPING_H

#include <string>
#include <memory>
#include <array>
#include <vector>

namespace SurgSim
{

namespace Graphics
{
class OsgMaterial;
}
namespace Framework
{
class SceneElement;
class Component;
}



namespace Blocks
{

///@{
/// Names to use as RenderGroupReferences
static const std::string GROUP_SHADOW_CASTER = "Shadowing";
static const std::string GROUP_SHADOW_RECEIVER = "Shadowed";
///@}

/// Builds a series of SceneElements enabling the rendering of shadows, all graphics object that should cast shadows
/// need to be in the render group GROUP_SHADOW_CASTER, all objects that should receive shadows should be in the
/// render group GROUP_SHADOW_RECEIVER. The rest is done by the graphics system.
/// All of the elements added are \sa RenderPass elements
/// \param camera the view camera that is used for this pass
/// \param light the light that should be used for the shadows
/// \param texture size the size of the render textures, gets allocated twice without blurring, four times if blurring
///        is enabled
/// \param lightCameraProjection parameters for an orthogonal projection that will be used to render the scene from
///        the lights point of view, needs to be set so it encompasses all the shadow casters and receivers
/// \param useBlur whether to blur the output of the light map pass, this will remove some of the blockiness of the
///        shadows
/// \param showDebug whether to show debug information
std::vector<std::shared_ptr<Framework::SceneElement>> createShadowMapping(
			std::shared_ptr<Framework::Component> camera,
			std::shared_ptr<Framework::Component> light,
			int textureSize,
			std::array<double, 6> lightCameraProjection,
			bool useBlur,
			bool showDebug);

}
}

#endif

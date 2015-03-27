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

#ifndef EXAMPLES_STAPLING_GRAPHICS_H
#define EXAMPLES_STAPLING_GRAPHICS_H

#include <unordered_map>
#include <memory>
#include <string>

#include <SurgSim/Graphics/OsgMaterial.h>

namespace SurgSim
{
namespace Framework
{
class Scene;
}
}
typedef std::unordered_map<std::string, std::shared_ptr<SurgSim::Graphics::OsgMaterial>> Materials;

Materials createMaterials(std::shared_ptr<SurgSim::Framework::Scene> scene);

void applyMaterials(std::shared_ptr<SurgSim::Framework::Scene> scene, Materials materials);

void setupShadowMapping(const std::unordered_map<std::string, std::shared_ptr<SurgSim::Graphics::OsgMaterial>>&
						materials, std::shared_ptr<SurgSim::Framework::Scene> scene);

#endif

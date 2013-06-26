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

#ifndef SURGSIM_GRAPHICS_SHADERBASE_H
#define SURGSIM_GRAPHICS_SHADERBASE_H

namespace SurgSim
{

namespace Graphics
{

/// Base class that defines the interface for graphics shaders.
///
/// Shaders are the programs executed on the GPU to render the scene geometry.
/// \todo	Define interface for loading the geometry/vertex/fragment shaders from files/strings.
class Shader
{
public:
	/// Destructor
	virtual ~Shader() = 0;
};

inline Shader::~Shader()
{
}

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_SHADERBASE_H

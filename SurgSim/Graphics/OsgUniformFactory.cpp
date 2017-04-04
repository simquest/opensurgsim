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

#include "SurgSim/Graphics/OsgUniformFactory.h"



#include "SurgSim/Graphics/OsgTexture1d.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgTexture3d.h"
#include "SurgSim/Graphics/OsgTextureCubeMap.h"
#include "SurgSim/Graphics/OsgUniform.h"

#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Graphics
{

OsgUniformFactory::OsgUniformFactory()
{
	// Scalar Types
	registerClass<OsgUniform<bool>>("bool");
	registerClass<OsgUniform<unsigned int>>("uint");
	registerClass<OsgUniform<int>>("int");
	registerClass<OsgUniform<float>>("float");
	registerClass<OsgUniform<double>>("double");

	// Vector Types
	registerClass<OsgUniform<SurgSim::Math::Vector2f>>("vec2");
	registerClass<OsgUniform<SurgSim::Math::Vector3f>>("vec3");
	registerClass<OsgUniform<SurgSim::Math::Vector4f>>("vec4");

	registerClass<OsgUniform<SurgSim::Math::Vector2d>>("dvec2");
	registerClass<OsgUniform<SurgSim::Math::Vector3d>>("dvec3");
	registerClass<OsgUniform<SurgSim::Math::Vector4d>>("dvec4");

	// Matrix Types
	registerClass<OsgUniform<SurgSim::Math::Matrix22f>>("mat2");
	registerClass<OsgUniform<SurgSim::Math::Matrix22f>>("mat2x2");
	registerClass<OsgUniform<SurgSim::Math::Matrix33f>>("mat3");
	registerClass<OsgUniform<SurgSim::Math::Matrix22f>>("mat3x3");
	registerClass<OsgUniform<SurgSim::Math::UnalignedMatrix44f>>("mat4");
	registerClass<OsgUniform<SurgSim::Math::Matrix22f>>("mat4x4");

	registerClass<OsgUniform<SurgSim::Math::Matrix22d>>("dmat2");
	registerClass<OsgUniform<SurgSim::Math::Matrix22f>>("dmat2x2");
	registerClass<OsgUniform<SurgSim::Math::Matrix33d>>("dmat3");
	registerClass<OsgUniform<SurgSim::Math::Matrix22f>>("dmat3x3");
	registerClass<OsgUniform<SurgSim::Math::Matrix44d>>("dmat4");
	registerClass<OsgUniform<SurgSim::Math::Matrix22f>>("dmat4x4");

	// Sampler Types
	registerClass<OsgTextureUniform<OsgTexture1d>>("sampler1D");
	registerClass<OsgTextureUniform<OsgTexture2d>>("sampler2D");
	registerClass<OsgTextureUniform<OsgTexture3d>>("sampler3D");
	registerClass<OsgTextureUniform<OsgTextureCubeMap>>("samplerCube");
}

OsgUniformFactory::~OsgUniformFactory()
{

}

}
}


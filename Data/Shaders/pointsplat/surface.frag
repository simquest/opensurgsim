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

/// \file surface.frag
/// Fragment Shader to render fluid surface
/// Most of the equations and concepts come from the nVidia
/// 'Screen Space Fluid Rendering' paper and presentation from GDC '10
/// found here: http://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf

#version 120

uniform sampler2D depthMap;
uniform sampler2D normalMap;
uniform mat4 inverseProjectionMatrix;
uniform float maxDepth = 0.999999f;
uniform vec3 lightDir = vec3(-0.5, 1.5, -5.);
uniform vec4 color;

vec3 getEyeSpacePos(vec2 texCoord, float z)
{
    vec2 homogenous = texCoord * 2.0 - 1.0;
	vec4 clipSpacePos = vec4(homogenous, z, 1.0);
	vec4 eyeSpacePos = inverseProjectionMatrix * clipSpacePos;
	return eyeSpacePos.xyz/eyeSpacePos.w;
}

void main(void)
{
    float depth = texture2D(depthMap, gl_TexCoord[0].xy).x;
    if (depth > maxDepth)
    {
        discard;
    }
    vec3 eyePos = getEyeSpacePos(gl_TexCoord[0].xy, depth);

    vec3 normal = normalize(texture2D(normalMap, gl_TexCoord[0].xy).xyz);

    // Need to use the vector from the vertex to the light, which is the negative of lightDir
    float diffuse = max(0.0,dot(normalize(-lightDir),normal));

    const float shininess = 100.0;
    float temp = max(dot(normalize(reflect(lightDir, normal)), normalize(-eyePos)), 0.0);
    float specular = temp / (shininess - temp * shininess + temp);

    // gl_FragColor = color * diffuse + specular;
	gl_FragColor.xyz = vec3(0.5,0.5, 0.0);
    gl_FragColor.a = 1.0;
    gl_FragDepth = depth;
}

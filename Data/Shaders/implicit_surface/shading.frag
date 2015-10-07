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

uniform sampler2D shadowMap;
uniform sampler2D depthMap;
uniform sampler2D normalMap;
uniform float shininess;

/// Main Camera Matrices
struct MainCamera
{
	mat4 viewMatrix;
	mat4 inverseViewMatrix;
	mat4 projectionMatrix;
	mat4 inverseProjectionMatrix;
};

uniform MainCamera mainCamera;

// Oss provided uniforms
uniform vec4 ambientColor;

// Incoming from the vertex shader
varying vec3 lightDir;

varying vec2 texCoord0; ///< Texture unit 0 texture coordinates
varying vec4 clipCoord; ///< Projected and transformed vertex coordinates

varying vec3 vertexDiffuseColor;
varying vec3 vertexSpecularColor;

vec3 getEyeSpacePos(vec2 texCoord, float z)
{
    vec2 homogenous = texCoord * 2.0 - 1.0;
	vec4 clipSpacePos = vec4(homogenous, z, 1.0);
	vec4 eyeSpacePos = mainCamera.inverseProjectionMatrix * clipSpacePos;
	return eyeSpacePos.xyz/eyeSpacePos.w;
}

void main(void)
{
    float maxDepth = 0.999999f;
    float depth = texture2D(depthMap, texCoord0).x;
    if (depth > maxDepth)
    {
        discard;
    }

    vec2 shadowCoord = clipCoord.xy / clipCoord.w * vec2(0.5) + vec2(0.5);

    float shadowAmount = 1.0 - texture2D(shadowMap, shadowCoord).r;
    vec3 vAmbient = ambientColor.xyz;

    vec3 normal = normalize(texture2D(normalMap, texCoord0).xyz * 2.0 - 1.0);

    vec3 lightDirNorm = normalize(lightDir);
    vec3 eyeDirNorm = normalize(getEyeSpacePos(texCoord0, depth));
    vec3 normalDirNorm = normalize(normal);

    float diffuse = max(dot(lightDirNorm, normalDirNorm), 0.0);
    vec3 vDiffuse = vertexDiffuseColor * diffuse;

    float temp = max(dot(reflect(lightDirNorm, normalDirNorm), eyeDirNorm), 0.0);
    float specular = temp / (shininess - temp * shininess + temp);

    vec3 vSpecular = vertexSpecularColor * specular;

    vec3 color = vAmbient + vDiffuse + vSpecular;

    gl_FragColor.rgb = color;
    gl_FragColor.a = 1.0;
    gl_FragDepth = depth;
}

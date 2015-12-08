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

// These are 'free' uniforms to be set for this shader, they won't be provided by OSS
uniform sampler2D shadowMap;
uniform sampler2D depthMap;
uniform sampler2D normalMap;
uniform samplerCube diffuseEnvMap;
uniform samplerCube specularEnvMap;
uniform vec4 diffuseColor;
uniform vec4 specularColor;
uniform float diffusePercent = 0.1;
uniform float specularPercent = 0.1;
uniform float shininess;

// Main Camera Matrices
struct MainCamera
{
	mat4 viewMatrix;
	mat4 inverseViewMatrix;
	mat4 projectionMatrix;
	mat4 inverseProjectionMatrix;
};

uniform MainCamera mainCamera;

struct LightSource {
	vec4 diffuse;
	vec4 specular;
	vec4 position;
	float constantAttenuation;
	float linearAttenuation;
	float quadraticAttenuation;
};

uniform LightSource lightSource;

// Oss provided uniforms
uniform vec4 ambientColor;

// Incoming from the vertex shader
varying vec2 texCoord0; ///< Texture unit 0 texture coordinates
varying vec4 clipCoord; ///< Projected and transformed vertex coordinates

vec3 getEyeSpacePos(vec3 coord)
{
    vec3 homogenous = coord * 2.0 - 1.0;
	vec4 clipSpacePos = vec4(homogenous, 1.0);
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

	vec4 eyeDir4 = vec4(getEyeSpacePos(vec3(texCoord0, depth)), 1.0);

	vec3 lightDir = (mainCamera.viewMatrix * lightSource.position - eyeDir4).xyz;
    float lightDistance = length(lightDir);

    float eyeDistance = length(eyeDir4.xyz);

    float attenuation = 1.0 / (lightSource.constantAttenuation + lightSource.linearAttenuation*lightDistance + lightSource.quadraticAttenuation*lightDistance*lightDistance);

    vec3 vertexDiffuseColor = (attenuation * diffuseColor * lightSource.diffuse).xyz;
	vec3 vertexSpecularColor = (attenuation * specularColor * lightSource.specular).xyz;

    vec2 shadowCoord = clipCoord.xy / clipCoord.w * vec2(0.5) + vec2(0.5);

    float shadowAmount = 1.0 - texture2D(shadowMap, shadowCoord).r;
    vec3 vAmbient = ambientColor.rgb * diffuseColor.rgb;

    vec3 normal = (texture2D(normalMap, texCoord0).xyz * 2.0) - 1.0;

    vec3 lightDirNorm = normalize(lightDir);
    vec3 eyeDirNorm = normalize(eyeDir4.xyz);
    vec3 normalDirNorm = normalize(normal);

    float diffuse = max(dot(lightDirNorm, normalDirNorm), 0.0);
    vec3 vDiffuse = vec3(textureCube(diffuseEnvMap,  normalDirNorm)) * vertexDiffuseColor * diffuse;

	vec3 color = mix(vAmbient, vDiffuse, diffusePercent);

    float temp = max(dot(reflect(lightDirNorm, normalDirNorm), eyeDirNorm), 0.0);
    float specular = temp / (shininess - temp * shininess + temp);
	vec3 reflectDir = reflect(eyeDirNorm, normalDirNorm);
    vec3 vSpecular = vec3(textureCube(specularEnvMap, reflectDir)) * vertexSpecularColor;

	color = mix(color, vSpecular + color, specularPercent) +
				(specular * specularColor * lightSource.specular * attenuation).rgb;

    gl_FragColor.rgb = color;
    gl_FragColor.a = 1.0;
    gl_FragDepth = depth;
}

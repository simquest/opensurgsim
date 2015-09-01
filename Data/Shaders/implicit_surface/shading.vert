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

/// \file surface.vert
/// Vertex Shader to render fluid surface
/// Most of the equations and concepts come from the nVidia
/// 'Screen Space Fluid Rendering' paper and presentation from GDC '10
/// found here: http://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf

#version 120

// These are 'free' uniforms to be set for this shader, they won't be provided by OSS
uniform vec4 diffuseColor;
uniform vec4 specularColor;

// Oss provided uniforms
uniform mat4 viewMatrix;

struct LightSource {
	vec4 diffuse;
	vec4 specular;
	vec4 position;
	float constantAttenuation;
	float linearAttenuation;
	float quadraticAttenuation;
};

uniform LightSource lightSource;

// Outgoing Values
// Normalized direction vector
varying vec3 lightDir;

varying vec2 texCoord0; ///< Texture unit 0 texture coordinates
varying vec4 clipCoord; ///< Projected and transformed vertex coordinates

varying vec3 vertexDiffuseColor;
varying vec3 vertexSpecularColor;


void main(void)
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	clipCoord = gl_Position;

	vec4 eyeDir4 = gl_ModelViewMatrix * gl_Vertex;

	texCoord0 = gl_MultiTexCoord0.xy;

	lightDir = ( viewMatrix * lightSource.position - eyeDir4).xyz;
    float lightDistance = length(lightDir);
	lightDir = normalize(lightDir);

    float eyeDistance = length(eyeDir4.xyz);

    float attenuation = 1.0 / (lightSource.constantAttenuation + lightSource.linearAttenuation*lightDistance + lightSource.quadraticAttenuation*lightDistance*lightDistance);

    vertexDiffuseColor = (attenuation * diffuseColor * lightSource.diffuse).xyz;
	vertexSpecularColor = (attenuation * specularColor * lightSource.specular).xyz;
}

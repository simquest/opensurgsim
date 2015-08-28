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

/// \file dns_mapping_material.frag
/// Phong material with diffuse, shadow and normal map

/// Material supplied values
uniform sampler2D diffuseMap;
uniform sampler2D normalMap;
uniform sampler2D shadowMap;

uniform float shininess;

/// Oss supplied values
uniform vec4 ambientColor;

/// Fragment shader supplied values
varying vec3 lightDir;
varying vec3 eyeDir;

varying vec2 texCoord0;
varying vec4 clipCoord;

varying vec3 vertexDiffuseColor;
varying vec3 vertexSpecularColor;

void calculateLigthing(
	in vec3 l, in vec3 n, in vec3 v, in float shininess, in float shadow,
	inout vec3 diffuse, inout vec3 specular)
{

	float dotNL = max(dot(l, n), 0.0);

	diffuse = diffuse * dotNL * shadow;

    float temp = max(dot(reflect(l, n), v), 0.0);
    float specularity = temp / (shininess - temp * shininess + temp);

    specular = specular * specularity * shadow;

}

void main(void) 
{	
	vec2 shadowCoord = clipCoord.xy / clipCoord.w * vec2(0.5) + vec2(0.5);
	float shadowAmount = 1.0 - texture2D(shadowMap, shadowCoord).r;

	vec3 normalDir = texture2D(normalMap, texCoord0).rgb * 2.0 - 1.0;
	normalDir.g = -normalDir.g;

	vec3 vAmbient = vec3(ambientColor);
    
	vec3 lightDirNorm = normalize(lightDir);
	vec3 eyeDirNorm = normalize(eyeDir);
	vec3 normalDirNorm = normalize(normalDir);

	vec3 base = texture2D(diffuseMap, texCoord0).rgb;

	vec3 vDiffuse = vertexDiffuseColor;
	vec3 vSpecular = vertexSpecularColor;

	calculateLigthing(lightDirNorm, normalDirNorm, eyeDirNorm, 
		shininess, shadowAmount, vDiffuse, vSpecular);

	vec3 color = vDiffuse * base + vSpecular;

	vDiffuse = vertexDiffuseColor;
	vSpecular = vertexSpecularColor;

	calculateLigthing(lightDirNorm, -normalDirNorm, eyeDirNorm, 
		shininess, shadowAmount, vDiffuse, vSpecular);

	color += vDiffuse * base + vSpecular;

	color += vAmbient * base;
	
	gl_FragColor.rgb = color;
	gl_FragColor.a = 1.0;
}
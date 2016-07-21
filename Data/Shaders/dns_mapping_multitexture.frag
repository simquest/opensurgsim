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

#version 120

/// \file dns_mapping_material.frag
/// Phong material with diffuse, shadow and normal map

/// Material supplied values
uniform sampler2D diffuseMap;
uniform sampler2D normalMap;
uniform sampler2D shadowMap;
uniform sampler2D paintMap;

uniform float shininess;

uniform float normalCorrection = -1.0;

/// Oss supplied values
struct LightSource {
	vec4 diffuse; 
	vec4 specular; 
	vec4 position; 
	float constantAttenuation; 
	float linearAttenuation; 
	float quadraticAttenuation;	
};

uniform LightSource lightSource;

/// Fragment shader supplied values
varying vec3 lightDir;
varying vec3 eyeDir;

varying vec2 texCoord0;
varying vec4 clipCoord;

varying vec3 vertexDiffuseColor;
varying vec3 vertexSpecularColor;
varying vec3 vertexAmbientColor;

void main(void) 
{	
	vec2 shadowCoord = clipCoord.xy / clipCoord.w * vec2(0.5) + vec2(0.5);
	float shadowAmount = 1.0 - texture2D(shadowMap, shadowCoord).r;

	vec3 normalDir = texture2D(normalMap, texCoord0).rgb * 2.0 - 1.0;
	normalDir.g =  normalCorrection * normalDir.g;
    
	vec3 lightDirNorm = normalize(lightDir);
	vec3 eyeDirNorm = normalize(eyeDir);
	vec3 normalDirNorm = normalize(normalDir);

	vec4 paint = texture2D(paintMap, texCoord0);

	float diffuse = max(dot(lightDirNorm, normalDirNorm), 0.0);
	
	vec3 vDiffuse = mix(vertexDiffuseColor * diffuse, paint.rgb, paint.a) * shadowAmount;
 
    float temp = max(dot(reflect(lightDirNorm, normalDirNorm), eyeDirNorm), 0.0);
    float specular = temp / (shininess - temp * shininess + temp);   
	vec3 vSpecular = vertexSpecularColor * specular * shadowAmount;		

	vec3 base = texture2D(diffuseMap, texCoord0).rgb;
	vec3 color = (vertexAmbientColor + vDiffuse) * base + vSpecular;

	gl_FragColor.rgb = color;
	gl_FragColor.a = 1.0;
}
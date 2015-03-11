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

// Material provided values
uniform vec4 diffuseColor;
uniform vec4 specularColor;

// OSS provided values
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

attribute vec3 tangent;
attribute vec3 bitangent;

/// Outgoing
varying vec3 lightDir;
varying vec3 eyeDir;

varying vec2 texCoord0;
varying vec4 clipCoord;

varying vec3 vertexDiffuseColor;
varying vec3 vertexSpecularColor;

void main(void) 
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

	clipCoord = gl_Position;
	
	vec4 eyeDir4 = gl_ModelViewMatrix * gl_Vertex;
	eyeDir = eyeDir4.xyz;
 
	texCoord0 = gl_MultiTexCoord0.xy;
	
	vec3 n = normalize(gl_NormalMatrix * gl_Normal);
	vec3 t = normalize(gl_NormalMatrix * tangent);
	vec3 b = normalize(gl_NormalMatrix * bitangent);
	
	vec3 v;
	vec3 l = (viewMatrix * lightSource.position).xyz - eyeDir4.xyz;
    float lightDistance = length(l);
	v.x = dot(l, t);
	v.y = dot(l, b);
	v.z = dot(l, n);
	lightDir = normalize(v);
	
    float eyeDistance = length(eyeDir);
	v.x = dot(eyeDir, t);
	v.y = dot(eyeDir, b);
	v.z = dot(eyeDir, n);
	eyeDir = normalize(v);
    
    float attenuation = 1.0 / (lightSource.constantAttenuation + lightSource.linearAttenuation*lightDistance + lightSource.quadraticAttenuation*lightDistance*lightDistance);
		
    vertexDiffuseColor = (attenuation * diffuseColor * lightSource.diffuse).xyz;	
	vertexSpecularColor = (attenuation * specularColor * lightSource.specular).xyz;
} 

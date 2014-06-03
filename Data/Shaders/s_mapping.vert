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

/// \file shadowmap_vertexcolor.vert
/// Vertex shader for simple default material, lighting from a single light source, per vertex

uniform mat4 viewMatrix;

/// outgoing calculated color for this vertex
varying vec4 color;

/// outgoing calculated projected coordinates for this vertex
varying vec4 clipCoord;

struct LightSource {
	vec4 ambient; 
	vec4 diffuse; 
	vec4 specular; 
	vec4 position; 
	float constantAttenuation; 
	float linearAttenuation; 
	float quadraticAttenuation;	
};

uniform LightSource lightSource;

void main(void) 
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	clipCoord = gl_Position;
	
	vec4 eyeDir4 = gl_ModelViewMatrix * gl_Vertex;
	vec3 eyeDir = normalize(eyeDir4.xyz);

	vec3 lightDir = (viewMatrix * lightSource.position).xyz - eyeDir4.xyz;
	float lightDistance = length(lightDir);
    lightDir = normalize(lightDir);
	
	vec3 normal = normalize(gl_NormalMatrix * gl_Normal);
    
    float attenuation = 1.0 / (lightSource.constantAttenuation + lightSource.linearAttenuation*lightDistance + 
		lightSource.quadraticAttenuation*lightDistance*lightDistance);
    
    color.rgb = attenuation * dot(lightDir, normal) * gl_Color.rgb * lightSource.diffuse.rgb;
	color.a = gl_Color.a;
} 

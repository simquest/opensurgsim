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

/// \file basic_lit.vert
/// Basic vertex shader with one light, only diffuse term is used for 
/// lighting, lighting is per vertex, light is considered to be a point light

varying vec4 color;

void main(void) 
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	
	vec4 eyeDir4 = gl_ModelViewMatrix * gl_Vertex;
	vec3 eyeDir = normalize(eyeDir4.xyz);

	vec3 lightDir = gl_LightSource[0].position.xyz - eyeDir4.xyz;
	float lightDistance = length(lightDir);
    lightDir = normalize(lightDir);
	
	vec3 normal = normalize(gl_NormalMatrix * gl_Normal);
    
    float attenuation = 1.0 / (gl_LightSource[0].constantAttenuation + gl_LightSource[0].linearAttenuation*lightDistance + 
		gl_LightSource[0].quadraticAttenuation*lightDistance*lightDistance);
    
    color.rgb = attenuation * dot(lightDir, normal) * gl_Color.rgb * gl_LightSource[0].diffuse.rgb + gl_LightSource[0].ambient.rgb;
	color.a = gl_Color.a;
} 

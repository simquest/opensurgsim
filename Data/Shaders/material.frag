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

/// \file material.frag
///  Phong material, using material colors

// These are 'free' uniforms to be set for this shader, they won't be provided by OSS
uniform float shininess;

// Incoming from the vertex shader
varying vec3 lightDir;
varying vec3 eyeDir;
varying vec3 normalDir;

varying vec2 texCoord0;

varying vec3 vertexDiffuseColor;
varying vec3 vertexSpecularColor;

varying vec4 clipCoord;

varying vec3 vAmbient;

void main(void) 
{	
    vec3 lightDirNorm = normalize(lightDir);
	vec3 eyeDirNorm = normalize(eyeDir);
	vec3 normalDirNorm = normalize(normalDir);

	float diffuse = max(dot(lightDirNorm, normalDirNorm), 0.0);
	vec3 vDiffuse = vertexDiffuseColor * diffuse;	
    
    float temp = max(dot(reflect(lightDirNorm, normalDirNorm), eyeDirNorm), 0.0);
    float specular = temp / (shininess - temp * shininess + temp);
 	vec3 vSpecular = vertexSpecularColor * specular;		
	
	vec3 color = vAmbient + vDiffuse + vSpecular;
    
	gl_FragColor.rgb = color;
	gl_FragColor.a = 1.0;
}
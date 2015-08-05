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

/// \file material.vert
///  Phong material, using material colors

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
// Normalized direction vectors
varying vec3 lightDir;
varying vec3 eyeDir;
varying vec3 normalDir;

varying vec2 texCoord0; ///< Texture unit 0 texture coordinates
varying vec4 clipCoord; ///< Projected and transformed vertex coordinates

varying vec3 vertexDiffuseColor;
varying vec3 vertexSpecularColor;


void main(void)
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	clipCoord = gl_Position;
	
	vec4 eyeDir4 = gl_ModelViewMatrix * gl_Vertex;
	eyeDir = eyeDir4.xyz;	
    
	texCoord0 = gl_MultiTexCoord0.xy;
	
	lightDir = (viewMatrix * lightSource.position - eyeDir4).xyz;
    float lightDistance = length(lightDir);
	lightDir = normalize(lightDir);
	
    float eyeDistance = length(eyeDir);
	eyeDir = normalize(eyeDir);
    
    // In the case of the curve gl_Normal carries the information about the segment
    // vector from this vertex to the next
    vec3 segmentDir = gl_NormalMatrix * gl_Normal;

    // Calculate the normal of the segment that is in the plane spanned between
    // lightDir and segmentDir, i.e. the normal that is closest to the light direction
    // still need to deal with the light and the segment dir being coincident
    if (abs(dot(lightDir, segmentDir)) < 0.0000001)
    {
	    normalDir = cross(vec3(1.0,1.0,1.0), segmentDir);
    }
    else
    {
	    normalDir = cross(lightDir, segmentDir);
    }

    normalDir = -cross(normalDir, segmentDir);
    
    float attenuation = 1.0 / (lightSource.constantAttenuation + lightSource.linearAttenuation*lightDistance + lightSource.quadraticAttenuation*lightDistance*lightDistance);
		
    vertexDiffuseColor = (attenuation * diffuseColor * lightSource.diffuse).xyz;	
	vertexSpecularColor = (attenuation * specularColor * lightSource.specular).xyz;
} 

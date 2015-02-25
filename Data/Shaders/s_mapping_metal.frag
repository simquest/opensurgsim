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

/// \file s_mapping_metal.frag
/// Metallic material, environment maps plus shadow maps
/// for use with s_mapping_metal.vert.
/// Ported from legacy SimQuest code, the origin of the material and lighting
/// calculation is unkown

// These are 'free' uniforms to be set for this shader, they won't be provided by OSS
uniform float specularPercent;
uniform float diffusePercent;
uniform float shininess;

uniform vec4 specularColor;
uniform samplerCube specularEnvMap;
uniform samplerCube diffuseEnvMap;
uniform sampler2D shadowMap;

// OSS provided uniforms
uniform vec4 ambientColor;

struct LightSource {
    vec4 diffuse; 
    vec4 specular; 
    vec4 position; 
    float constantAttenuation; 
    float linearAttenuation; 
    float quadraticAttenuation; 
};

uniform LightSource lightSource;

// From Vertex Shader
varying vec3 lightDir;
varying vec3 eyeDir;
varying vec3 reflectDir;
varying vec3 normalDir;

varying vec4 clipCoord;

varying float attenuation;

void main()
{
	// Calculate Shadowing
	vec2 shadowCoord = clipCoord.xy / clipCoord.w * vec2(0.5) + vec2(0.5);
	float shadowAmount = 1.0 - texture2D(shadowMap, shadowCoord).r;

	// Look up environment map values in cube maps
	vec3 mappedDiffuseColor = vec3(textureCube(diffuseEnvMap,  normalDir)) * 
			(lightSource.diffuse * attenuation * shadowAmount).rgb;
	vec3 mappedSpecularColor = vec3(textureCube(specularEnvMap, reflectDir)) *
			(lightSource.specular * attenuation).rgb;

	// Add lighting to base color and mix
	vec3 color = mix(ambientColor.rgb, mappedDiffuseColor, diffusePercent);

	vec3 normalizedLightDir = normalize(lightDir);
	vec3 normalizedNormalDir = normalize(normalDir);
	vec3 normalizedEyeDir = normalize(eyeDir);

	float temp = max(dot(reflect(normalizedLightDir, normalizedNormalDir), normalizedEyeDir), 0.0);
	float specular = temp / (shininess - temp * shininess + temp);

	color = mix(color, mappedSpecularColor + color, specularPercent) + 
			(specular * specularColor * lightSource.specular * attenuation * shadowAmount).rgb;

	gl_FragColor.rgb = color;
	gl_FragColor.a = 1.0;
}


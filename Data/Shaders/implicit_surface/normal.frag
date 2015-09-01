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

/// \file sphere_normal.frag
/// Fragment Shader to calculate normals of a point sprite sphere generated surface
/// Most of the equations and concepts come from the nVidia
/// Screen Space Fluid Rendering paper and presentation from GDC '10
/// found here: http://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf

#version 120

uniform sampler2D depthMap;
uniform float maxDepth = 0.999999f;
uniform mat4 mainInverseProjectionMatrix;
uniform float texelSize = 1.0/1024.0;

vec3 getEyeSpacePos(vec2 texCoord, float z)
{
	vec2 homogenous = texCoord * 2.0 - 1.0;
	vec4 clipSpacePos = vec4(homogenous, z, 1.0);
	vec4 eyeSpacePos = mainInverseProjectionMatrix * clipSpacePos;
	return eyeSpacePos.xyz/eyeSpacePos.w;
}

void main(void)
{
	float depth = texture2D(depthMap, gl_TexCoord[0].xy).x;

	if(depth > maxDepth)
	{
		discard;
	}

	vec3 eyePos = getEyeSpacePos(gl_TexCoord[0].xy, depth);
	vec2 texCoordX1 = gl_TexCoord[0].xy + vec2(texelSize, 0);
	vec2 texCoordX2 = gl_TexCoord[0].xy - vec2(texelSize, 0);

	vec3 ddx = getEyeSpacePos(texCoordX1, texture2D(depthMap, texCoordX1).x) - eyePos;
	vec3 ddx2 = eyePos - getEyeSpacePos(texCoordX2, texture2D(depthMap, texCoordX2).x);
	if(abs(ddx.z) > abs(ddx2.z))
	{
		ddx = ddx2;
	}

	vec2 texCoordY1 = gl_TexCoord[0].xy + vec2(0, texelSize);
	vec2 texCoordY2 = gl_TexCoord[0].xy - vec2(0, texelSize);

	vec3 ddy = getEyeSpacePos(texCoordY1, texture2D(depthMap, texCoordY1).x) - eyePos;
	vec3 ddy2 = eyePos - getEyeSpacePos(texCoordY2, texture2D(depthMap, texCoordY2).x);
	if(abs(ddy.z) > abs(ddy2.z))
	{
		ddy = ddy2;
	}

	vec3 normal = cross(ddx, ddy);
	normal = normalize(normal);
	//normal = 0.5f*(normal+1.0f);

	gl_FragColor = vec4(normal, 1.0);
}

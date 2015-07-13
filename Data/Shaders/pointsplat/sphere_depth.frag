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

/// \file sphere.frag
/// Fragment Shader to do simple point sprite spheres

#version 120

varying vec3 eyeSpacePos;
varying mat4 projectionMatrix;
varying float sphereRadius;

void main(void)
{
	// calculate normal from texture coordinates provided by gl_PointCoord
	vec3 N;
	N.xy = gl_PointCoord * 2.0 - vec2(1.0);
	float mag = dot(N.xy, N.xy);
	if (mag > 1.0) discard;   // kill pixels outside circle
	N.z = sqrt(1.0-mag);

	// calculate depth
	vec4 pixelPos = vec4(eyeSpacePos + N*sphereRadius, 1.0);
	vec4 clipSpacePos = gl_ProjectionMatrix * pixelPos;
	gl_FragDepth = clipSpacePos.z / clipSpacePos.w;
}

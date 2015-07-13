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

/// \file sphere.vert
/// Vertex Shader to do simple point sprite spheres

#version 120

uniform vec2 screenSize;
uniform float sphereRadius;

void main(void)
{
	vec4 eyePos = gl_ModelViewMatrix * gl_Vertex;
	vec4 projVoxel = gl_ProjectionMatrix * vec4(sphereRadius, sphereRadius, eyePos.z, eyePos.w);
	vec2 projSize = screenSize * projVoxel.xy / projVoxel.w;
	gl_PointSize = 0.25 * (projSize.x + projSize.y);
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}

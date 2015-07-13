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

uniform float sphereRadius;

varying mat4 projectionMatrix;
varying vec3 eyeSpacePos;

void main(void)
{
	projectionMatrix = gl_ProjectionMatrix;
	vec3 eyeSpacePos = vec3(gl_ModelViewMatrix * vec4(gl_Vertex.xyz, 1.0));
	float dist = length(eyeSpacePos);
	gl_PointSize = sphereRadius * (0.25 / dist);
	gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1.0);
}

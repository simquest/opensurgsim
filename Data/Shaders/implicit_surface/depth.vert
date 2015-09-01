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

/// \file sphere_depth.vert
/// Vertex Shader to calculate depth of point sprite spheres
/// Most of the equations and concepts come from the nVidia
/// Screen Space Fluid Rendering paper and presentation from GDC '10
/// found here: http://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf

#version 120

uniform float sphereRadius;
uniform float sphereScale;

varying vec3 eyeSpacePos;

void main(void)
{
	eyeSpacePos = vec3(gl_ModelViewMatrix * gl_Vertex);
	float dist = length(eyeSpacePos);
	gl_PointSize = sphereRadius * (sphereScale / dist);
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}

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
/// Most of the equations and concepts come from the nVidia
/// Screen Space Fluid Rendering paper and presentation from GDC '10
/// found here: http://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf

#version 120

uniform vec4 color;

void main(void)
{
	// calculate normal from texture coordinates provided by gl_PointCoord
	vec3 normal;
	normal.xy = gl_PointCoord * 2.0 - vec2(1.0);
	float mag = dot(normal.xy, normal.xy);
	if (mag > 1.0)
	{
		discard; // kill pixels outside circle
	}
	normal.z = sqrt(1.0-mag);

	gl_FragColor = color * normal.z;
}

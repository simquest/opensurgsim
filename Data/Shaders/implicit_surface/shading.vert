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

/// \file surface.vert
/// Vertex Shader to render fluid surface
/// Most of the equations and concepts come from the nVidia
/// 'Screen Space Fluid Rendering' paper and presentation from GDC '10
/// found here: http://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf

#version 120

// Outgoing Values
varying vec2 texCoord0; ///< Texture unit 0 texture coordinates
varying vec4 clipCoord; ///< Projected and transformed vertex coordinates

void main(void)
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	clipCoord = gl_Position;

	texCoord0 = gl_MultiTexCoord0.xy;
}

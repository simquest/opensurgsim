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

/// \file shadow_map.vert
/// Calculate a shadow map that can be used for modulating the color of each
/// fragment by the amount of shadow that should be applied to that color

/// inverse view matrix from the rendering camera
uniform mat4 inverseViewMatrix;

/// view matrix for the camera that rendered the original depth map (see shadow_map.frag)
uniform mat4 lightViewMatrix;

/// projection matrix for the camera that rendered the original depth map (see shadow_map.frag)
uniform mat4 lightProjectionMatrix;

varying vec4 lightCoord;

void main(void) 
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

	// compute the coordinates of the incoming point in the space of the camera used
	// to render the depth map
	lightCoord = lightProjectionMatrix * lightViewMatrix * inverseViewMatrix * gl_ModelViewMatrix * gl_Vertex;
} 
 
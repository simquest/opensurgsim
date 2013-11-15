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

/// \file shadowMapping.frag
/// Vertex Shader, moves the point into the coordinate system of the depth map generated
/// by depth.[vert|frag]

uniform mat4 oss_inverseViewMatrix;
uniform mat4 oss_lightViewMatrix;
uniform mat4 oss_lightProjectionMatrix;

varying vec4 lightCoord;

void main(void) 
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

	//compute light coord
    lightCoord = oss_lightProjectionMatrix * oss_lightViewMatrix * oss_inverseViewMatrix * gl_ModelViewMatrix *  gl_Vertex;
} 
 
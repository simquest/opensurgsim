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

uniform float height;
uniform float blurRadius;

varying vec2 taps[7];
varying float weights[7];

void main(void) 
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    
    vec2 texCoord0 = gl_MultiTexCoord0.xy;
    
    float dy = (blurRadius / 3.0) / height;
    
    taps[0] = texCoord0 - vec2(0.0, -3.0) * dy;
    taps[1] = texCoord0 - vec2(0.0, -2.0) * dy;
    taps[2] = texCoord0 - vec2(0.0, -1.0) * dy;
    taps[3] = texCoord0 - vec2(0.0, 0.0) * dy;
    taps[4] = texCoord0 - vec2(0.0, 1.0) * dy;
    taps[5] = texCoord0 - vec2(0.0, 2.0) * dy;
    taps[6] = texCoord0 - vec2(0.0, 3.0) * dy;

    weights[0] = 0.04779035227281;
    weights[1] = 0.11086490165864;
    weights[2] = 0.21078608625031;
    weights[3] = 0.26111731963647;
    weights[4] = 0.21078608625031;
    weights[5] = 0.11086490165864;
    weights[6] = 0.04779035227281;
} 
 
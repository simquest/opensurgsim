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

/// \file horizontalBlurPass.vert
/// Vertex Shader, for a simple 2-pass Blur horizontal pass

/// Width of the Texture that is incoming
uniform float width;

/// Sampling radius
uniform float blurRadius;

varying vec2 taps[7];

void main(void) 
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    
    vec2 texCoord0 = gl_MultiTexCoord0.xy;
    
    float dx = (blurRadius / 3.0) / width;
    
    taps[0] = texCoord0 - vec2(-3.0, 0.0) * dx;
    taps[1] = texCoord0 - vec2(-2.0, 0.0) * dx;
    taps[2] = texCoord0 - vec2(-1.0, 0.0) * dx;
    taps[3] = texCoord0 - vec2(0.0, 0.0) * dx;
    taps[4] = texCoord0 - vec2(1.0, 0.0) * dx;
    taps[5] = texCoord0 - vec2(2.0, 0.0) * dx;
    taps[6] = texCoord0 - vec2(3.0, 0.0) * dx;
} 
 

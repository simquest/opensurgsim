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

/// \file horizontalBlurPass.frag
/// Fragment Shader, for a simple 2-pass Blur horizontal pass

/// Use version 120 for const array capability
#version 120

/// Texture to sample, e.g. on a ScreenSpaceQuad
uniform sampler2D texture;

/// Texture coordinates for sampling
varying vec2 taps[7];

/// Fixed weights for blur
const float weights[7] = float[](
    0.04779035227281,
    0.11086490165864,
    0.21078608625031,
    0.26111731963647,
    0.21078608625031,
    0.11086490165864,
    0.04779035227281);

void main(void) 
{	
    vec4 sampledValue = vec4(0.0);
    
    sampledValue += weights[0] * texture2D(texture, taps[0]);
    sampledValue += weights[1] * texture2D(texture, taps[1]);
    sampledValue += weights[2] * texture2D(texture, taps[2]);
    sampledValue += weights[3] * texture2D(texture, taps[3]);
    sampledValue += weights[4] * texture2D(texture, taps[4]);
    sampledValue += weights[5] * texture2D(texture, taps[5]);
    sampledValue += weights[6] * texture2D(texture, taps[6]);
    
	gl_FragColor.rgba = sampledValue;
}

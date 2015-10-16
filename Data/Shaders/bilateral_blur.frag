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

/// \file bilateral_blur.frag
/// Fragment Shader, for a bilateral filtering blur, used in both the
/// vertical and horizontal pass

/// Use version 120 for const array capability
#version 120

/// Texture to sample, e.g. on a ScreenSpaceQuad
uniform sampler2D texture;

/// Texture coordinates for sampling
varying vec2 texCoord0;
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
    float depth = texture2D(texture, texCoord0).x;
    float result = 0.0;
    float normalization = 0.0;

        for (int i = 0; i < 7; i++)
        {
            float sample = texture2D(texture, taps[i]).x;
            float gaussian = weights[i];

            float closeness = distance(sample, depth); // length(vec3(1,1,1));

            if(closeness < 0.6)
            {
                float sampleWeight = exp(-closeness * closeness) * gaussian;

                result += sample * sampleWeight;
                normalization += sampleWeight;
            }
            else
            {
                result += depth * gaussian;
                normalization += gaussian;
            }
        }

	gl_FragDepth = result / normalization;
}

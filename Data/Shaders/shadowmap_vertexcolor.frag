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

/// \file shadowmap_vertexcolor.frag
/// Modulate the outgoing color by the amount fetched from the oss_shadowMap, intended for use
/// with simple vertex colors, does not do any texturing

/// map for modulation, needs to be rendered with the same modelview and projection
/// matrices as the ones that are used for this shader.
uniform sampler2D oss_shadowMap;

/// incoming color for this fragment
varying vec4 color;

/// incoming projected coordinates of the current fragment
varying vec4 clipCoord;

void main(void)
{
	vec2 shadowCoord = clipCoord.xy / clipCoord.w * vec2(0.5) + vec2(0.5);
	float shadowAmount = 1.0 - texture2D(oss_shadowMap, shadowCoord).r;
	gl_FragColor = color * shadowAmount;
}

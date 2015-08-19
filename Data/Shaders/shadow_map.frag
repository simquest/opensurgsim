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

/// \file shadow_map.frag
/// Calculate a shadow map that can be used for modulating the color of each
/// fragment by the amount of shadow that should be applied to that color,
/// the outgoing image will have indicate the amount of shadowing from 1 .. 0
/// 1 being fully in the shadow, 0 being full light

/// Texture rendered from the view of the light, containing the distance of each fragment
/// from the light, the z value of the incoming fragment projected into
/// light space is close to the value in the texture map then the fragment is lit, otherwise
/// it is not
uniform sampler2D depthMap;


/// The coordinates of the fragment in the space of the projected depth map
varying vec4 lightCoord;

void main(void)
{
	// Calculate texture coordinates from incoming point
	vec3 lightCoord3 = (lightCoord.xyz / lightCoord.w) * vec3(0.5) + vec3(0.5);

	float depth = texture2D(depthMap, lightCoord3.xy);

	gl_FragColor = vec4(depth + 0.01 > lightCoord3.z ? 0.0 : 1.0);
}

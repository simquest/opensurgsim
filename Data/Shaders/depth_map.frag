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

/// \file depth.frag
/// Encode the z-depth of the fragment into rgba values, see
/// http://www.ozone3d.net/blogs/lab/20080604/glsl-float-to-rgba8-encoder/

vec4 encodeDepth(float depth)
{
	vec4 encoded = vec4(1.0, 256.0, 256.0*256.0, 256.0*256.0*256.0) * depth;
	encoded = fract(encoded);
	encoded -= encoded.yzww * vec4(1.0/256.0, 1.0/256.0, 1.0/256.0, 0.0);
	return encoded;
}

void main(void) 
{	
	gl_FragColor.rgba = encodeDepth(gl_FragCoord.z);
}
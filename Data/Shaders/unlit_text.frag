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

/// \file unlit_text.frag
/// Fragment Shader to do text rendering
/// use with unlit_texture.vert

uniform sampler2D texture;

varying vec2 texCoord0;
varying vec4 color;

void main(void) 
{	
	gl_FragColor.rgb = color.rgb;
	gl_FragColor.a = texture2D(texture, texCoord0.st).a; 
}
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


// Oss provided uniforms
uniform mat4 viewMatrix;

struct LightSource {
    vec4 diffuse; 
    vec4 specular; 
    vec4 position; 
    float constantAttenuation; 
    float linearAttenuation; 
    float quadraticAttenuation; 
};

uniform LightSource lightSource;


varying vec3 lightDir;
varying vec3 eyeDir;
varying vec3 reflectDir;
varying vec3 normalDir;

varying vec4 clipCoord;

varying float attenuation;

void main() 
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

    clipCoord = gl_Position;

    vec4 eyeDir4 = gl_ModelViewMatrix * gl_Vertex;

    lightDir = (viewMatrix * lightSource.position).xyz - eyeDir4.xyz;

    float lightDistance = length(lightDir);
    float eyeDistance = length(eyeDir4.xyz);

    lightDir = normalize(lightDir);
    eyeDir = normalize(eyeDir4.xyz);

    normalDir = normalize(gl_NormalMatrix * gl_Normal);
    vec4 pos = gl_ModelViewMatrix * gl_Vertex;
    reflectDir = reflect(pos.xyz, normalDir);

    attenuation = 1.0 / (lightSource.constantAttenuation + 
        lightSource.linearAttenuation*lightDistance + 
        lightSource.quadraticAttenuation*lightDistance*lightDistance);
}

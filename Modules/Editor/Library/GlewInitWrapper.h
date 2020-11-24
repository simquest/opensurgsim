// This file is a part of the OpenSurgSim project.
// Copyright 2020, SimQuest Solutions Inc.
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

#ifndef SURGSIM_EDITDEBUG_GLEWINITWRAPPER_H
#define SURGSIM_EDITDEBUG_GLEWINITWRAPPER_H


/// Wraps glewInit() to separate the glew opengl definitions from the osg opengl definitions
/// only imgui needs glew but we need to call glewInit() from a osg callback, using this call
/// we avoid getting warnings about redefinitions
namespace SurgSim
{
namespace EditDebug
{

	int ossGlewInit();

}
}

#endif
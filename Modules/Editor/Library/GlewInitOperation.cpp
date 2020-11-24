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

#include "GlewInitOperation.h"
#include "GlewInitWrapper.h"

#include "SurgSim/Framework/Assert.h"

#include <osg/GraphicsContext>
#include "backends/imgui_impl_opengl3.h"

namespace SurgSim {
namespace EditDebug {

GlewInitOperation::GlewInitOperation() : osg::Operation("GlewInitCallback", false)
{

}

void GlewInitOperation::operator()(osg::Object* object)
{
	osg::GraphicsContext* context = dynamic_cast<osg::GraphicsContext*>(object);
	if (!context)
		return;

	if (ossGlewInit() != 0)
	{
		SURGSIM_FAILURE() << "glewInit() failed";
	}
	ImGui_ImplOpenGL3_Init();
}

}
}



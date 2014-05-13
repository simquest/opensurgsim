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

#include "SurgSim/Graphics/View.h"

namespace SurgSim
{
namespace Graphics
{




View::View(const std::string& name) :
	SurgSim::Framework::Component(name)
{

}

bool View::setCamera(std::shared_ptr<Camera> camera)
{
	m_camera = camera;
	return true;
}

std::shared_ptr<Camera> View::getCamera() const
{
	return m_camera;
}

bool View::doInitialize()
{
	SURGSIM_ASSERT(m_camera != nullptr) << "View cannot be created without a camera.";
	return true;
}

}
}


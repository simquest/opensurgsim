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

#ifndef SURGSIM_EDITDEBUG_VISUALDEBUGBEHAVIOR_H
#define SURGSIM_EDITDEBUG_VISUALDEBUGBEHAVIOR_H

#include "SurgSim/Framework/Behavior.h"

namespace SurgSim
{
namespace Graphics
{
	class Camera;
	class View;
}
}

namespace SurgSim
{
namespace EditDebug
{

SURGSIM_STATIC_REGISTRATION(VisualDebugBehavior)

class VisualDebugBehavior : public SurgSim::Framework::Behavior
{
public:

	friend class DebugGui;

	SURGSIM_CLASSNAME("SurgSim::EditDebug::VisualDebugBehavior");

	VisualDebugBehavior(const std::string& name);

	int getTargetManagerType() const override;
	
	bool doInitialize() override;
	bool doWakeUp() override;
	void update(double dt) override;
private:

	std::shared_ptr<SurgSim::Graphics::View> m_view;
	std::shared_ptr<SurgSim::Graphics::Camera> m_camera;
};

}
}



#endif
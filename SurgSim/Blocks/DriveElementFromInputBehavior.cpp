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

#include "SurgSim/Blocks/DriveElementFromInputBehavior.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Blocks
{

DriveElementFromInputBehavior::DriveElementFromInputBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_poseName("pose")
{
}

void DriveElementFromInputBehavior::setFrom(std::shared_ptr<SurgSim::Input::InputComponent> from)
{
	m_from = from;
}

void DriveElementFromInputBehavior::setPoseName(const std::string& poseName)
{
	m_poseName = poseName;
}

void DriveElementFromInputBehavior::update(double dt)
{
	SurgSim::DataStructures::DataGroup dataGroup;
	m_from->getData(&dataGroup);
	RigidTransform3d pose;
	if (dataGroup.poses().get(m_poseName, &pose))
	{
		getSceneElement()->setPose(pose);
	}
}

bool DriveElementFromInputBehavior::doInitialize()
{
	return true;
}

bool DriveElementFromInputBehavior::doWakeUp()
{
	bool result = true;
	if (m_from == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << "DriveElementFromInputBehavior named '" +
			getName() + "' must have a driver to do anything.";
		result = false;
	}

	return result;
}

}; //namespace Blocks
}; //namespace SurgSim

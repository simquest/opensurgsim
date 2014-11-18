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
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Math::RigidTransform3d;
using SurgSim::Framework::checkAndConvert;

namespace SurgSim
{
namespace Blocks
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::DriveElementFromInputBehavior,
				 DriveElementFromInputBehavior);

DriveElementFromInputBehavior::DriveElementFromInputBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_poseName("pose")
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(DriveElementFromInputBehavior, std::shared_ptr<SurgSim::Framework::Component>,
									  Source, getSource, setSource);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(DriveElementFromInputBehavior, std::string, PoseName, getPoseName, setPoseName);
}

void DriveElementFromInputBehavior::setSource(std::shared_ptr<SurgSim::Framework::Component> source)
{
	m_source = checkAndConvert<SurgSim::Input::InputComponent>(source, "SurgSim::Input::InputComponent");
}

std::shared_ptr<SurgSim::Framework::Component> DriveElementFromInputBehavior::getSource()
{
	return m_source;
}

void DriveElementFromInputBehavior::setPoseName(const std::string& poseName)
{
	m_poseName = poseName;
}

std::string DriveElementFromInputBehavior::getPoseName()
{
	return m_poseName;
}

void DriveElementFromInputBehavior::update(double dt)
{
	SurgSim::DataStructures::DataGroup dataGroup;
	m_source->getData(&dataGroup);
	RigidTransform3d pose;
	if (dataGroup.poses().get(m_poseName, &pose))
	{
		getPoseComponent()->setPose(m_source->getLocalPose() * pose);
	}
}

bool DriveElementFromInputBehavior::doInitialize()
{
	return true;
}

bool DriveElementFromInputBehavior::doWakeUp()
{
	bool result = true;
	if (m_source == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << getClassName() << " named '" +
				getName() + "' must have a source to do anything.";
		result = false;
	}

	if (getPoseComponent() == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << getClassName() << " named '" +
				getName() + "' must belong to a SceneElement with a PoseComponent.";
		result = false;
	}

	return result;
}

}; //namespace Blocks
}; //namespace SurgSim

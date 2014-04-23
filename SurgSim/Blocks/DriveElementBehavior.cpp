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

#include "SurgSim/Blocks/DriveElementBehavior.h"

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Physics/Representation.h"

namespace
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::DriveElementBehavior);
}

namespace SurgSim
{
namespace Blocks
{

DriveElementBehavior::DriveElementBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(DriveElementBehavior, std::shared_ptr<SurgSim::Framework::Component>,
			Source, getSource, setSource);
}

void DriveElementBehavior::setSource(std::shared_ptr<SurgSim::Framework::Component> source)
{
	m_source = std::dynamic_pointer_cast<SurgSim::Physics::Representation>(source);
	SURGSIM_ASSERT(m_source != nullptr) << " setSource for " << getClassName()
		<< " requires a SurgSim::Physics::Representation";
}

std::shared_ptr<SurgSim::Framework::Component> DriveElementBehavior::getSource()
{
	return m_source;
}

void DriveElementBehavior::update(double dt)
{
	m_source->driveElement();
}

bool DriveElementBehavior::doInitialize()
{
	return true;
}

bool DriveElementBehavior::doWakeUp()
{
	bool result = true;
	if (m_source == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << getClassName() << " named '" +
			getName() + "' must have a driver to do anything.";
		result = false;
	}

	return result;
}

}; //namespace Blocks
}; //namespace SurgSim

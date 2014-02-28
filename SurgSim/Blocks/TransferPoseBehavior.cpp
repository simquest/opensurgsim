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

#include "SurgSim/Blocks/TransferPoseBehavior.h"

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Representation.h"

namespace SurgSim
{
namespace Blocks
{

TransferPoseBehavior::TransferPoseBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name)
{
}

void TransferPoseBehavior::setPoseSender(std::shared_ptr<SurgSim::Framework::Representation> sender)
{
	m_from = sender;
}

void TransferPoseBehavior::setPoseReceiver(std::shared_ptr<SurgSim::Framework::Representation> receiver)
{
	m_to = receiver;
}

void TransferPoseBehavior::update(double dt)
{
	m_to->setPose(m_from->getPose());
}

bool TransferPoseBehavior::doInitialize()
{
	return true;
}

bool TransferPoseBehavior::doWakeUp()
{
	if (m_to == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << "TransferPoseBehavior named '" +
			getName() + "' must have a receiver to do anything.";
		return false;
	}

	if (m_from == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << "TransferPoseBehavior named '" +
			getName() + "' must have a sender to do anything.";
		return false;
	}
	
	m_to->setInitialPose(m_from->getInitialPose());
	return true;
}

}; //namespace Blocks
}; //namespace SurgSim

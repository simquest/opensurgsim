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

#include "SurgSim/Blocks/TransferInputPoseBehavior.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Blocks
{

TransferInputPoseBehavior::TransferInputPoseBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_from(),
	m_to(),
	m_poseName("pose")
{
}

void TransferInputPoseBehavior::setPoseFrom(std::shared_ptr<SurgSim::Input::InputComponent> from)
{
	m_from = from;
}

void TransferInputPoseBehavior::setPoseTo(std::shared_ptr<SurgSim::Framework::Representation> to)
{
	m_to = to;
}

void TransferInputPoseBehavior::setPoseName(const std::string& poseName)
{
	m_poseName = poseName;
}

void TransferInputPoseBehavior::update(double dt)
{
	SurgSim::DataStructures::DataGroup dataGroup;
	m_from->getData(&dataGroup);
	RigidTransform3d pose;
	dataGroup.poses().get(m_poseName, &pose);
	m_to->setPose(pose);
}

bool TransferInputPoseBehavior::doInitialize()
{
	return true;
}

bool TransferInputPoseBehavior::doWakeUp()
{
	return true;
}

}; //namespace Blocks
}; //namespace SurgSim

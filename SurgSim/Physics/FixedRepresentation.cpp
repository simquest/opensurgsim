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

#include "SurgSim/Physics/FixedRepresentation.h"

namespace SurgSim
{
namespace Physics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::FixedRepresentation, FixedRepresentation);

FixedRepresentation::FixedRepresentation(const std::string& name) :
	RigidRepresentationBase(name)
{
}

FixedRepresentation::~FixedRepresentation()
{
}


std::string FixedRepresentation::getType() const
{
	return getClassName();
}

void FixedRepresentation::updateGlobalInertiaMatrices(const RigidRepresentationState& state)
{
	// Do Nothing it is a fixed object
}

void FixedRepresentation::update(double dt)
{
	m_currentState.setPose(getPose());
}

}; // Physics
}; // SurgSim
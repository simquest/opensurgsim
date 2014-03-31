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

#include "SurgSim/Physics/ConstraintComponent.h"

namespace SurgSim
{
namespace Physics
{

ConstraintComponent::ConstraintComponent(const std::string& name) : Component(name)
{
}

ConstraintComponent::~ConstraintComponent()
{
}

void ConstraintComponent::setConstraint(std::shared_ptr<Constraint> constraint)
{
	m_constraint = constraint;
}

std::shared_ptr<Constraint> ConstraintComponent::getConstraint() const
{
	return m_constraint;
}

bool ConstraintComponent::doInitialize()
{
	return m_constraint != nullptr;
};

bool ConstraintComponent::doWakeUp()
{
	return true;
}

}; // namespace Physics
}; // namespace SurgSim

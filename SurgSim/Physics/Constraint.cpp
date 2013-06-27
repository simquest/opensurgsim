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

#include <SurgSim/Physics/Constraint.h>

namespace SurgSim
{

namespace Physics
{

Constraint::Constraint()
{
}

Constraint::Constraint(std::shared_ptr<ConstraintImplementation> side0,
                       std::shared_ptr<ConstraintImplementation> side1)
{
	m_implementations = std::make_pair(side0, side1);
}

Constraint::~Constraint()
{
}

void Constraint::doBuild(double dt,
	const ConstraintData& data,
	MlcpPhysicsProblem &mlcp,
	unsigned int offsetActor0,
	unsigned int offsetActor1,
	unsigned int offsetConstraint)
{
	if (m_implementations.first)
	{
		m_implementations.first->build(dt, data, mlcp, offsetActor0, offsetConstraint, CONSTRAINT_POSITIVE_SIDE);
	}
	if (m_implementations.second)
	{
		m_implementations.second->build(dt, data, mlcp, offsetActor1, offsetConstraint, CONSTRAINT_NEGATIVE_SIDE);
	}
}

void Constraint::doClear()
{
}

}; // namespace Physics

}; // namespace SurgSim

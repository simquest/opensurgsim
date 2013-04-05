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

#include "Constraint.h"

#include <SurgSim/Physics/Constraints/ConstraintData.h>
#include <SurgSim/Physics/Constraints/ConstraintImplementation.h>

using SurgSim::Physics::Constraint;
using SurgSim::Physics::ConstraintData;
using SurgSim::Physics::ConstraintImplementation;

Constraint::Constraint()
{
}
Constraint::Constraint(std::shared_ptr<ConstraintImplementation> side0,
                       std::shared_ptr<ConstraintImplementation> side1)
{
	m_sides[0] = side0;
	m_sides[1] = side1;
}
Constraint::~Constraint()
{
}

void Constraint::doBuild(const ConstraintData& data,
                         const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor>& a,
                         const Eigen::Matrix<double, Eigen::Dynamic, 1,  Eigen::DontAlign>& b)
{
	m_sides[0]->build(data, a, b);
	m_sides[1]->build(data, a, b);
}

void Constraint::doReset()
{
}
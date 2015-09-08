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

#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/ConstraintImplementation.h"

namespace SurgSim
{

namespace Physics
{

ConstraintImplementation::ConstraintImplementation()
{
}

ConstraintImplementation::~ConstraintImplementation()
{
}

ConstraintImplementationFactory& ConstraintImplementation::getFactory()
{
	static ConstraintImplementationFactory factory;
	return factory;
}

size_t ConstraintImplementation::getNumDof() const
{
	return doGetNumDof();
}

void ConstraintImplementation::build(double dt,
	const ConstraintData& data,
	const std::shared_ptr<Localization>& localization,
	MlcpPhysicsProblem* mlcp,
	size_t indexOfRepresentation,
	size_t indexOfConstraint,
	ConstraintSideSign sign)
{
	doBuild(dt, data, localization, mlcp, indexOfRepresentation, indexOfConstraint, sign);
}

}; // namespace Physics

}; // namespace SurgSim

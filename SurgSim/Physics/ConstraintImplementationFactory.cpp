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

#include "SurgSim/Physics/ConstraintImplementationFactory.h"
#include "SurgSim/Physics/FixedRepresentationContact.h"
#include "SurgSim/Physics/RigidRepresentationContact.h"

namespace SurgSim
{
namespace Physics
{

ConstraintImplementationFactory::ConstraintImplementationFactory()
{
	addImplementation(std::make_shared<FixedRepresentationContact>());
	addImplementation(std::make_shared<RigidRepresentationContact>());
}

ConstraintImplementationFactory::~ConstraintImplementationFactory()
{
}

std::shared_ptr<ConstraintImplementation> ConstraintImplementationFactory::getImplementation(
	RepresentationType representationType,
	SurgSim::Math::MlcpConstraintType constraintType) const
{
	SURGSIM_ASSERT(representationType >= 0 && representationType < REPRESENTATION_TYPE_COUNT) <<
		"Invalid representation type " << representationType;
	SURGSIM_ASSERT(constraintType >= 0 && constraintType < SurgSim::Math::MLCP_NUM_CONSTRAINT_TYPES) <<
		"Invalid constraint type " << constraintType;

	return m_implementations[representationType][constraintType];
}

void ConstraintImplementationFactory::addImplementation(std::shared_ptr<ConstraintImplementation> implementation)
{
	m_implementations[implementation->getRepresentationType()][implementation->getMlcpConstraintType()] =
		implementation;
}

}; // Physics
}; // SurgSim

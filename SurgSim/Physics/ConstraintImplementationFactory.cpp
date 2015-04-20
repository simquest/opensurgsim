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

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Physics/ConstraintImplementationFactory.h"
#include "SurgSim/Physics/Fem3DConstraintBilateral3D.h"
#include "SurgSim/Physics/Fem3DConstraintContact.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FixedConstraintBilateral3D.h"
#include "SurgSim/Physics/FixedConstraintContact.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidConstraintBilateral3D.h"
#include "SurgSim/Physics/RigidConstraintContact.h"
#include "SurgSim/Physics/RigidRepresentation.h"

namespace SurgSim
{
namespace Physics
{

ConstraintImplementationFactory::ConstraintImplementationFactory()
{
	addImplementation(typeid(FixedRepresentation), std::make_shared<FixedConstraintContact>());
	addImplementation(typeid(RigidRepresentation), std::make_shared<RigidConstraintContact>());
	addImplementation(typeid(Fem3DRepresentation), std::make_shared<Fem3DConstraintContact>());
	addImplementation(typeid(FixedRepresentation), std::make_shared<FixedConstraintBilateral3D>());
	addImplementation(typeid(RigidRepresentation), std::make_shared<RigidConstraintBilateral3D>());
	addImplementation(typeid(Fem3DRepresentation), std::make_shared<Fem3DConstraintBilateral3D>());
}

ConstraintImplementationFactory::~ConstraintImplementationFactory()
{
}

std::shared_ptr<ConstraintImplementation> ConstraintImplementationFactory::getImplementation(
	std::type_index representationType,
	SurgSim::Math::MlcpConstraintType constraintType)
{
	SURGSIM_ASSERT(constraintType >= 0 && constraintType < SurgSim::Math::MLCP_NUM_CONSTRAINT_TYPES) <<
		"Invalid constraint type " << constraintType;

	auto implementation = m_implementations[representationType][constraintType];
	SURGSIM_LOG_IF(implementation == nullptr, SurgSim::Framework::Logger::getDefaultLogger(), WARNING) <<
		"No constraint implementation for representation type (" << representationType.name() <<
		") and constraint type (" << constraintType << ")";

	return implementation;
}

void ConstraintImplementationFactory::addImplementation(
	std::type_index typeIndex, std::shared_ptr<ConstraintImplementation> implementation)
{
	m_implementations[typeIndex][implementation->getMlcpConstraintType()] =
		implementation;
}

}; // Physics
}; // SurgSim

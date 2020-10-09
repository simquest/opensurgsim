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

#include "SurgSim/Blocks/MassSpring1DRepresentation.h"
#include "SurgSim/Blocks/MassSpring2DRepresentation.h"
#include "SurgSim/Blocks/MassSpring3DRepresentation.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Physics/ConstraintImplementationFactory.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemConstraintFixedPoint.h"
#include "SurgSim/Physics/FemConstraintFixedRotationVector.h"
#include "SurgSim/Physics/FemConstraintFrictionalSliding.h"
#include "SurgSim/Physics/FemConstraintFrictionlessContact.h"
#include "SurgSim/Physics/FemConstraintFrictionlessSliding.h"
#include "SurgSim/Physics/FixedConstraintFixedPoint.h"
#include "SurgSim/Physics/FixedConstraintFixedRotationVector.h"
#include "SurgSim/Physics/FixedConstraintFrictionlessContact.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/MassSpringConstraintFixedPoint.h"
#include "SurgSim/Physics/MassSpringConstraintFrictionlessContact.h"
#include "SurgSim/Physics/MassSpringConstraintFrictionalSliding.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/RigidConstraintFixedPoint.h"
#include "SurgSim/Physics/RigidConstraintFixedRotationVector.h"
#include "SurgSim/Physics/RigidConstraintFrictionlessContact.h"
#include "SurgSim/Physics/RigidRepresentation.h"

namespace SurgSim
{
namespace Physics
{

ConstraintImplementationFactory::ConstraintImplementationFactory()
{
	using SurgSim::Blocks::MassSpring1DRepresentation;
	using SurgSim::Blocks::MassSpring2DRepresentation;
	using SurgSim::Blocks::MassSpring3DRepresentation;

	addImplementation(typeid(FixedRepresentation), std::make_shared<FixedConstraintFrictionlessContact>());
	addImplementation(typeid(RigidRepresentation), std::make_shared<RigidConstraintFrictionlessContact>());
	addImplementation(typeid(Fem1DRepresentation), std::make_shared<FemConstraintFrictionlessContact>());
	addImplementation(typeid(Fem2DRepresentation), std::make_shared<FemConstraintFrictionlessContact>());
	addImplementation(typeid(Fem3DRepresentation), std::make_shared<FemConstraintFrictionlessContact>());
	addImplementation(typeid(FixedRepresentation), std::make_shared<FixedConstraintFixedPoint>());
	addImplementation(typeid(RigidRepresentation), std::make_shared<RigidConstraintFixedPoint>());
	addImplementation(typeid(Fem1DRepresentation), std::make_shared<FemConstraintFixedPoint>());
	addImplementation(typeid(Fem2DRepresentation), std::make_shared<FemConstraintFixedPoint>());
	addImplementation(typeid(Fem3DRepresentation), std::make_shared<FemConstraintFixedPoint>());
	addImplementation(typeid(Fem1DRepresentation), std::make_shared<FemConstraintFrictionlessSliding>());
	addImplementation(typeid(Fem2DRepresentation), std::make_shared<FemConstraintFrictionlessSliding>());
	addImplementation(typeid(Fem3DRepresentation), std::make_shared<FemConstraintFrictionlessSliding>());
	addImplementation(typeid(Fem1DRepresentation), std::make_shared<FemConstraintFrictionalSliding>());
	addImplementation(typeid(Fem2DRepresentation), std::make_shared<FemConstraintFrictionalSliding>());
	addImplementation(typeid(Fem3DRepresentation), std::make_shared<FemConstraintFrictionalSliding>());

	addImplementation(typeid(FixedRepresentation), std::make_shared<FixedConstraintFixedRotationVector>());
	addImplementation(typeid(RigidRepresentation), std::make_shared<RigidConstraintFixedRotationVector>());
	addImplementation(typeid(Fem1DRepresentation), std::make_shared<FemConstraintFixedRotationVector>());

	addImplementation(typeid(MassSpringRepresentation), std::make_shared<MassSpringConstraintFrictionlessContact>());
	addImplementation(typeid(MassSpringRepresentation), std::make_shared<MassSpringConstraintFixedPoint>());
	addImplementation(typeid(MassSpringRepresentation), std::make_shared<MassSpringConstraintFrictionalSliding>());
	addImplementation(typeid(MassSpring1DRepresentation), std::make_shared<MassSpringConstraintFrictionlessContact>());
	addImplementation(typeid(MassSpring1DRepresentation), std::make_shared<MassSpringConstraintFixedPoint>());
	addImplementation(typeid(MassSpring1DRepresentation), std::make_shared<MassSpringConstraintFrictionalSliding>());
	addImplementation(typeid(MassSpring2DRepresentation), std::make_shared<MassSpringConstraintFrictionlessContact>());
	addImplementation(typeid(MassSpring2DRepresentation), std::make_shared<MassSpringConstraintFixedPoint>());
	addImplementation(typeid(MassSpring2DRepresentation), std::make_shared<MassSpringConstraintFrictionalSliding>());
	addImplementation(typeid(MassSpring3DRepresentation), std::make_shared<MassSpringConstraintFrictionlessContact>());
	addImplementation(typeid(MassSpring3DRepresentation), std::make_shared<MassSpringConstraintFixedPoint>());
}

ConstraintImplementationFactory::~ConstraintImplementationFactory()
{
}

std::shared_ptr<ConstraintImplementation> ConstraintImplementationFactory::getImplementation(
		std::type_index representationType, ConstraintType constraintType)
{
	SURGSIM_ASSERT(constraintType >= 0 && constraintType < NUM_CONSTRAINT_TYPES) <<
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
	m_implementations[typeIndex][implementation->getConstraintType()] =
		implementation;
}

}; // Physics
}; // SurgSim

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

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Physics/Fem3DRepresentationContact.h"
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
	addImplementation(std::make_shared<Fem3DRepresentationContact>());
}

ConstraintImplementationFactory::~ConstraintImplementationFactory()
{
}

std::shared_ptr<ConstraintImplementation> ConstraintImplementationFactory::getImplementation(
	const std::string& representationType,
	SurgSim::Math::MlcpConstraintType constraintType) const
{
	SURGSIM_ASSERT(representationType.size() > 0) <<
		"Representation type cannot be empty";
	SURGSIM_ASSERT(constraintType >= 0 && constraintType < SurgSim::Math::MLCP_NUM_CONSTRAINT_TYPES) <<
		"Invalid constraint type " << constraintType;

	std::shared_ptr<ConstraintImplementation> implementation;
	auto representationImplementations = m_implementations.find(representationType);
	if (representationImplementations != m_implementations.end())
	{
		auto constraintImplementation = representationImplementations->second.find(constraintType);
		if (constraintImplementation != representationImplementations->second.end())
		{
			implementation = constraintImplementation->second;
		}
	}

	SURGSIM_LOG_IF(implementation == nullptr, SurgSim::Framework::Logger::getDefaultLogger(), WARNING) <<
			"No constraint implementation for representation type (" << representationType <<
			") and constraint type (" << constraintType << ")";

	return implementation;
}

void ConstraintImplementationFactory::addImplementation(std::shared_ptr<ConstraintImplementation> implementation)
{
// 	std::unordered_map<SurgSim::Math::MlcpConstraintType,
// 		std::shared_ptr<ConstraintImplementation>>& representationImplementations =
// 		m_implementations[implementation->getRepresentationType()];
// 	representationImplementations[implementation->getMlcpConstraintType()] = implementation;
	//m_implementations[implementation->getRepresentationType()] = representationImplementations;
	m_implementations[implementation->getRepresentationType()][implementation->getMlcpConstraintType()] =
		implementation;
}

}; // Physics
}; // SurgSim

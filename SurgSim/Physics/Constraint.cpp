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
#include "SurgSim/Physics/Localization.h"

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{

namespace Physics
{

Constraint::Constraint(
	std::shared_ptr<ConstraintData> data,
	std::shared_ptr<ConstraintImplementation> implementation0,
	std::shared_ptr<Localization> localization0,
	std::shared_ptr<ConstraintImplementation> implementation1,
	std::shared_ptr<Localization> localization1)
{
	setInformation(data, implementation0, localization0, implementation1, localization1);
}

Constraint::~Constraint()
{
}


const std::pair<std::shared_ptr<ConstraintImplementation>, std::shared_ptr<ConstraintImplementation>>&
	Constraint::getImplementations() const
{
	return m_implementations;
}

const std::pair<std::shared_ptr<Localization>, std::shared_ptr<Localization>>& Constraint::getLocalizations() const
{
	return m_localizations;
}


std::shared_ptr<ConstraintData> Constraint::getData() const
{
	return m_data;
}

unsigned int Constraint::getNumDof() const
{
	return m_numDof;
}

SurgSim::Math::MlcpConstraintType Constraint::getType()
{
	return m_constraintType;
}

void Constraint::build(double dt,
	MlcpPhysicsProblem* mlcp,
	unsigned int indexOfRepresentation0,
	unsigned int indexOfRepresentation1,
	unsigned int indexOfConstraint)
{
	doBuild(dt, *m_data.get(), mlcp, indexOfRepresentation0, indexOfRepresentation1, indexOfConstraint);

	m_implementations.first->build(
		dt,
		*m_data.get(),
		m_localizations.first,
		mlcp,
		indexOfRepresentation0,
		indexOfConstraint,
		CONSTRAINT_POSITIVE_SIDE);

	m_implementations.second->build(
		dt,
		*m_data.get(),
		m_localizations.second,
		mlcp,
		indexOfRepresentation1,
		indexOfConstraint,
		CONSTRAINT_NEGATIVE_SIDE);

	mlcp->constraintTypes.push_back(m_constraintType);
}

void Constraint::doBuild(double dt,
	const ConstraintData& data,
	MlcpPhysicsProblem* mlcp,
	unsigned int indexOfRepresentation0,
	unsigned int indexOfRepresentation1,
	unsigned int indexOfConstraint)
{
}

void Constraint::setInformation(
	std::shared_ptr<ConstraintData> data,
	std::shared_ptr<ConstraintImplementation> implementation0,
	std::shared_ptr<Localization> localization0,
	std::shared_ptr<ConstraintImplementation> implementation1,
	std::shared_ptr<Localization> localization1)
{
	SURGSIM_ASSERT(data != nullptr) << "ConstraintData can't be nullptr";
	SURGSIM_ASSERT(implementation0 != nullptr) << "First implementation can't be nullptr";
	SURGSIM_ASSERT(localization0 != nullptr) << "First localization can't be nullptr";
	SURGSIM_ASSERT(implementation1 != nullptr) << "Second implementation can't be nullptr";
	SURGSIM_ASSERT(localization1 != nullptr) << "Second localization can't be nullptr";
	SURGSIM_ASSERT(implementation0->getMlcpConstraintType() != SurgSim::Math::MLCP_INVALID_CONSTRAINT) <<
		"First implementation has an invalid constraint type";
	SURGSIM_ASSERT(implementation1->getMlcpConstraintType() != SurgSim::Math::MLCP_INVALID_CONSTRAINT) <<
		"Second implementation has an invalid constraint type";
	SURGSIM_ASSERT(implementation0->getMlcpConstraintType() == implementation1->getMlcpConstraintType()) <<
		"Implementations have incompatible implementations first( " << implementation0->getMlcpConstraintType() <<
		" != " << implementation1->getMlcpConstraintType() << " )";
	SURGSIM_ASSERT(implementation0->getNumDof() == implementation1->getNumDof()) <<
		"Both sides of the constraint should have the same number of Dof ("<< m_implementations.first->getNumDof() <<
		" != " << m_implementations.second->getNumDof() <<")";


	m_data = data;
	m_implementations = std::make_pair(implementation0, implementation1);
	m_localizations = std::make_pair(localization0, localization1);
	m_numDof = implementation0->getNumDof();
	m_constraintType = implementation0->getMlcpConstraintType();
}

}; // namespace Physics

}; // namespace SurgSim

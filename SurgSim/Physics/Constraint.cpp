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

#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/Localization.h"

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{

namespace Physics
{

Constraint::Constraint(ConstraintType constraintType,
	std::shared_ptr<ConstraintData> data,
	std::shared_ptr<Representation> representation0,
	const SurgSim::DataStructures::Location& location0,
	std::shared_ptr<Representation> representation1,
	const SurgSim::DataStructures::Location& location1)
	: m_active(true)
{
	m_mlcpMap[FIXED_3DPOINT] = Math::MLCP_BILATERAL_3D_CONSTRAINT;
	m_mlcpMap[FIXED_3DROTATION_VECTOR] = Math::MLCP_BILATERAL_3D_CONSTRAINT;
	m_mlcpMap[FRICTIONAL_3DCONTACT] = Math::MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT;
	m_mlcpMap[FRICTIONLESS_3DCONTACT] = Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;
	m_mlcpMap[FRICTIONAL_SLIDING] = Math::MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT;
	m_mlcpMap[FRICTIONLESS_SLIDING] = Math::MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT;
	setInformation(constraintType, data, representation0, location0, representation1, location1);
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

size_t Constraint::getNumDof() const
{
	return m_numDof;
}

ConstraintType Constraint::getType()
{
	return m_constraintType;
}

void Constraint::build(double dt,
	MlcpPhysicsProblem* mlcp,
	size_t indexOfRepresentation0,
	size_t indexOfRepresentation1,
	size_t indexOfConstraint)
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

	mlcp->constraintTypes.push_back(
				(m_constraintType != INVALID_CONSTRAINT) ? m_mlcpMap[m_constraintType] : Math::MLCP_INVALID_CONSTRAINT);
}

bool Constraint::isActive()
{
	return m_active && m_localizations.first->getRepresentation()->isActive() &&
		   m_localizations.second->getRepresentation()->isActive();
}

void Constraint::setActive(bool flag)
{
	m_active = flag;
}

void Constraint::doBuild(double dt,
	const ConstraintData& data,
	MlcpPhysicsProblem* mlcp,
	size_t indexOfRepresentation0,
	size_t indexOfRepresentation1,
	size_t indexOfConstraint)
{
}

void Constraint::setInformation(ConstraintType constraintType,
	std::shared_ptr<ConstraintData> data,
	std::shared_ptr<Representation> representation0,
	const SurgSim::DataStructures::Location& location0,
	std::shared_ptr<Representation> representation1,
	const SurgSim::DataStructures::Location& location1)
{
	m_constraintType = constraintType;
	SURGSIM_ASSERT(data != nullptr) << "ConstraintData can't be nullptr";
	SURGSIM_ASSERT(representation0 != nullptr) << "First representation can't be nullptr";
	SURGSIM_ASSERT(representation1 != nullptr) << "Second representation can't be nullptr";

	auto localization0 = representation0->createLocalization(location0);
	SURGSIM_ASSERT(localization0 != nullptr) << "Could not create localization for " << representation0->getName();

	auto localization1 = representation1->createLocalization(location1);
	SURGSIM_ASSERT(localization1 != nullptr) << "Could not create localization for " << representation1->getName();

	auto implementation0 = representation0->getConstraintImplementation(m_constraintType);
	SURGSIM_ASSERT(implementation0 != nullptr) << "Could not get implementation for " << representation0->getName();

	auto implementation1 = representation1->getConstraintImplementation(m_constraintType);
	SURGSIM_ASSERT(implementation1 != nullptr) << "Could not get implementation for " << representation1->getName();

	SURGSIM_ASSERT(implementation0->getNumDof() == implementation1->getNumDof())
		<< "The number of DOFs does not match between the two implementations";

	m_data = data;
	m_implementations = std::make_pair(implementation0, implementation1);
	m_localizations = std::make_pair(localization0, localization1);
	m_numDof = implementation0->getNumDof();
}

}; // namespace Physics

}; // namespace SurgSim

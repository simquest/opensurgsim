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
#include <SurgSim/Physics/ConstraintData.h>

#include <SurgSim/Framework/Assert.h>

namespace SurgSim
{

namespace Physics
{

Constraint::Constraint(std::shared_ptr<ConstraintImplementation> side0,
                       std::shared_ptr<ConstraintImplementation> side1)
{
	SURGSIM_ASSERT(side0 != nullptr && side1 != nullptr) << "A constraint needs 2 valid constraint implementations!";

	m_implementations = std::make_pair(side0, side1);
}

Constraint::~Constraint()
{
}

void Constraint::setImplementations(std::shared_ptr<ConstraintImplementation> side0,
	std::shared_ptr<ConstraintImplementation> side1)
{
	SURGSIM_ASSERT(side0 != nullptr && side1 != nullptr) << "A constraint needs 2 valid constraint implementations!";

	m_implementations = std::make_pair(side0, side1);
}

const std::pair<std::shared_ptr<ConstraintImplementation>, std::shared_ptr<ConstraintImplementation>>&
	Constraint::getImplementations() const
{
	return m_implementations;
}

void Constraint::setData(std::shared_ptr<ConstraintData> data)
{
	m_data = data;
}

std::shared_ptr<ConstraintData> Constraint::getData() const
{
	return m_data;
}

unsigned int Constraint::getNumDof() const
{
	SURGSIM_ASSERT(m_implementations.first->getNumDof() == m_implementations.second->getNumDof()) <<
		"Both sides of the constraint should have the same number of Dof ("<< m_implementations.first->getNumDof() <<
		" != " << m_implementations.second->getNumDof() <<")" << std::endl;

	return m_implementations.first->getNumDof();
}

void Constraint::build(double dt,
	MlcpPhysicsProblem &mlcp,
	unsigned int indexOfRepresentation0,
	unsigned int indexOfRepresentation1,
	unsigned int indexOfConstraint)
{
	SURGSIM_ASSERT(m_data.get() != nullptr) << "Constraint data has not been set for this constraint." << std::endl;

	SurgSim::Math::MlcpConstraintType mlcpConstraintType = SurgSim::Math::MLCP_INVALID_CONSTRAINT;

	doBuild(dt, *m_data.get(), mlcp, indexOfRepresentation0, indexOfRepresentation1, indexOfConstraint);

	if (m_implementations.first)
	{
		m_implementations.first->build(dt, *m_data.get(), mlcp, indexOfRepresentation0, indexOfConstraint,
			CONSTRAINT_POSITIVE_SIDE);
		mlcpConstraintType = m_implementations.first->getMlcpConstraintType();
	}

	if (m_implementations.second)
	{
		m_implementations.second->build(dt, *m_data.get(), mlcp, indexOfRepresentation1, indexOfConstraint,
			CONSTRAINT_NEGATIVE_SIDE);
		SurgSim::Math::MlcpConstraintType mlcpConstraintType_2nd = m_implementations.second->getMlcpConstraintType();
		if (mlcpConstraintType == SurgSim::Math::MLCP_INVALID_CONSTRAINT)
		{
			mlcpConstraintType = mlcpConstraintType_2nd;
		}
		SURGSIM_ASSERT(mlcpConstraintType == mlcpConstraintType_2nd) <<
			"A constraint has 2 incompatible implementations:( " << mlcpConstraintType << " , " <<
			mlcpConstraintType_2nd << " )" << std::endl;
	}

	mlcp.constraintTypes.push_back(mlcpConstraintType);
}

void Constraint::doBuild(double dt,
	const ConstraintData& data,
	MlcpPhysicsProblem &mlcp,
	unsigned int indexOfRepresentation0,
	unsigned int indexOfRepresentation1,
	unsigned int indexOfConstraint)
{
}

}; // namespace Physics

}; // namespace SurgSim

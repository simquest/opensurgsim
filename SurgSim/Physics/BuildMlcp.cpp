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

#include <Eigen/Core>
using Eigen::MatrixXd;
using Eigen::VectorXd;

#include "SurgSim/Physics/BuildMlcp.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Physics
{

BuildMlcp::BuildMlcp(bool doCopyState) : Computation(doCopyState)
{}

BuildMlcp::~BuildMlcp()
{}

std::shared_ptr<PhysicsManagerState>
	BuildMlcp::doUpdate(const double& dt, const std::shared_ptr<PhysicsManagerState>& state)
{
	// Copy state to new state
	std::shared_ptr<PhysicsManagerState> result = state;
	std::vector<std::shared_ptr<Representation>> representations = result->getRepresentations();
	result->updateRepresentationsMapping();
	result->updateConstraintsMapping();

	size_t numAtomicConstraint = 0;
	size_t numConstraint = 0;
	size_t numDof = 0;

	// Calculate numAtomicConstraint and numConstraint
	for (int constraintType = 0;
		constraintType < CONSTRAINT_GROUP_TYPE_COUNT;
		constraintType++)
	{
		ConstraintGroupType constraintGroupType = static_cast<ConstraintGroupType>(constraintType);
		auto const constraints = result->getConstraintGroup(constraintGroupType);
		for (auto it = constraints.begin(); it != constraints.end(); it++)
		{
			if ((*it)->isActive())
			{
				numAtomicConstraint += (*it)->getNumDof();
				numConstraint++;
			}
		}
	}

	// Calculate numDof size
	for (auto it = result->getRepresentations().begin();
		it != result->getRepresentations().end();
		it++)
	{
		if ((*it)->isActive())
		{
			numDof += (*it)->getNumDof();
		}
	}

	// Resize the Mlcp problem
	result->getMlcpProblem().A.resize(numAtomicConstraint, numAtomicConstraint);
	result->getMlcpProblem().A.setZero();
	result->getMlcpProblem().b.resize(numAtomicConstraint);
	result->getMlcpProblem().b.setZero();
	result->getMlcpProblem().H.resize(numAtomicConstraint, numDof);
	result->getMlcpProblem().H.setZero();
	result->getMlcpProblem().CHt.resize(numDof, numAtomicConstraint);
	result->getMlcpProblem().CHt.setZero();
	result->getMlcpProblem().mu.resize(numConstraint);
	result->getMlcpProblem().mu.setZero();
	result->getMlcpProblem().constraintTypes.clear();

	// Resize the Mlcp solution
	result->getMlcpSolution().dofCorrection.resize(numDof);
	result->getMlcpSolution().dofCorrection.setZero();
	result->getMlcpSolution().x.resize(numAtomicConstraint);
	result->getMlcpSolution().x.setZero();

	// Fill up the Mlcp problem
	size_t constraintId = 0;
	for (int constraintType = 0;
		constraintType < CONSTRAINT_GROUP_TYPE_COUNT;
		constraintType++)
	{
		ConstraintGroupType constraintGroupType = static_cast<ConstraintGroupType>(constraintType);
		auto const constraints = result->getConstraintGroup(constraintGroupType);
		for (auto it = constraints.begin(); it != constraints.end(); it++)
		{
			if (!(*it)->isActive())
			{
				continue;
			}

			ptrdiff_t indexConstraint = result->getConstraintsMapping().getValue((*it).get());
			SURGSIM_ASSERT(indexConstraint >= 0) << "Index for constraint is invalid: " << indexConstraint << std::endl;

			std::shared_ptr<ConstraintImplementation> side0 = (*it)->getImplementations().first;
			std::shared_ptr<ConstraintImplementation> side1 = (*it)->getImplementations().second;
			SURGSIM_ASSERT(side0) << "Constraint does not have a side[0]" << std::endl;
			SURGSIM_ASSERT(side1) << "Constraint does not have a side[1]" << std::endl;
			std::shared_ptr<Localization> localization0 = (*it)->getLocalizations().first;
			std::shared_ptr<Localization> localization1 = (*it)->getLocalizations().second;
			SURGSIM_ASSERT(localization0) << "ConstraintImplementation does not have a localization on side[0]";
			SURGSIM_ASSERT(localization1) << "ConstraintImplementation does not have a localization on side[1]";
			const MlcpMapping<Representation>& mapping = result->getRepresentationsMapping();
			ptrdiff_t indexRepresentation0 = mapping.getValue(localization0->getRepresentation().get());
			ptrdiff_t indexRepresentation1 = mapping.getValue(localization1->getRepresentation().get());
			SURGSIM_ASSERT(indexRepresentation0 >= 0) << "Index for representation 0 is invalid: " <<
				indexRepresentation0;
			SURGSIM_ASSERT(indexRepresentation1 >= 0) << "Index for representation 0 is invalid: " <<
				indexRepresentation1;

			(*it)->build(dt, &result->getMlcpProblem(), indexRepresentation0, indexRepresentation1, indexConstraint);

			constraintId++;
		}
	}

	return result;
}


}; // namespace Physics
}; // namespace SurgSim

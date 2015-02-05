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
	MlcpMapping<Representation> representationsMapping;
	MlcpMapping<Constraint> constraintsMapping;

	size_t numAtomicConstraint = 0;
	size_t numConstraint = 0;
	size_t numDof = 0;

	// Calculate numAtomicConstraint and numConstraint
	auto const activeConstraints = result->getActiveConstraints();
	numConstraint = activeConstraints.size();
	for (auto it = activeConstraints.cbegin(); it != activeConstraints.cend(); it++)
	{
		constraintsMapping.setValue((*it).get(), static_cast<ptrdiff_t>(numAtomicConstraint));
		numAtomicConstraint += (*it)->getNumDof();
	}
	result->setConstraintsMapping(constraintsMapping);

	// Calculate numDof size
	auto const activeRepresentations = result->getActiveRepresentations();
	for (auto it = activeRepresentations.cbegin(); it != activeRepresentations.cend(); it++)
	{
		representationsMapping.setValue((*it).get(), numDof);
		numDof += (*it)->getNumDof();
	}
	result->setRepresentationsMapping(representationsMapping);

	// Resize the Mlcp problem
	result->getMlcpProblem().A.setZero(numAtomicConstraint, numAtomicConstraint);
	result->getMlcpProblem().b.setZero(numAtomicConstraint);
	result->getMlcpProblem().H.setZero(numAtomicConstraint, numDof);
	result->getMlcpProblem().CHt.setZero(numDof, numAtomicConstraint);
	result->getMlcpProblem().mu.setZero(numConstraint);
	result->getMlcpProblem().constraintTypes.clear();

	// Resize the Mlcp solution
	result->getMlcpSolution().dofCorrection.setZero(numDof);
	result->getMlcpSolution().x.setZero(numAtomicConstraint);

	// Fill up the Mlcp problem
	for (auto it = activeConstraints.begin(); it != activeConstraints.end(); it++)
	{
		ptrdiff_t indexConstraint = result->getConstraintsMapping().getValue((*it).get());
		SURGSIM_ASSERT(indexConstraint >= 0) << "Index for constraint is invalid: " << indexConstraint << std::endl;

		std::shared_ptr<ConstraintImplementation> side0 = (*it)->getImplementations().first;
		std::shared_ptr<ConstraintImplementation> side1 = (*it)->getImplementations().second;
		SURGSIM_ASSERT(side0) << "Constraint does not have a side0" << std::endl;
		SURGSIM_ASSERT(side1) << "Constraint does not have a side1" << std::endl;
		std::shared_ptr<Localization> localization0 = (*it)->getLocalizations().first;
		std::shared_ptr<Localization> localization1 = (*it)->getLocalizations().second;
		SURGSIM_ASSERT(localization0) << "ConstraintImplementation does not have a localization on side0";
		SURGSIM_ASSERT(localization1) << "ConstraintImplementation does not have a localization on side1";
		const MlcpMapping<Representation>& mapping = result->getRepresentationsMapping();
		ptrdiff_t indexRepresentation0 = mapping.getValue(localization0->getRepresentation().get());
		ptrdiff_t indexRepresentation1 = mapping.getValue(localization1->getRepresentation().get());
		SURGSIM_ASSERT(indexRepresentation0 >= 0) << "Index for representation 0 is invalid: " <<
			indexRepresentation0;
		SURGSIM_ASSERT(indexRepresentation1 >= 0) << "Index for representation 1 is invalid: " <<
			indexRepresentation1;

		(*it)->build(dt, &result->getMlcpProblem(), indexRepresentation0, indexRepresentation1, indexConstraint);
	}

	return result;
}


}; // namespace Physics
}; // namespace SurgSim

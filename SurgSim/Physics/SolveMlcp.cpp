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

#include "SurgSim/Physics/SolveMlcp.h"

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

namespace SurgSim
{
namespace Physics
{

SolveMlcp::SolveMlcp(bool doCopyState) : Computation(doCopyState)
{
}

SolveMlcp::~SolveMlcp()
{}

std::shared_ptr<PhysicsManagerState> SolveMlcp::doUpdate(const double& dt,
		const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;

	// Solve the Mlcp using a Gauss-Seidel solver
	m_gaussSeidelSolver.solve(result->getMlcpProblem(), &(result->getMlcpSolution()));

	// lambda
	const Eigen::VectorXd& lambda = result->getMlcpSolution().x;
	if ((lambda.size() == 0) || !result->getMlcpSolution().validConvergence)
	{
		return result;
	}

	// Copy constraintForces back to contact
	auto& constraintsMapping = result->getConstraintsMapping();
	const auto& activeConstraints = result->getActiveConstraints();

	for (auto& constraint : activeConstraints)
	{
		if (constraint->getType() != ConstraintType::FRICTIONLESS_3DCONTACT)
		{
			continue;
		}

		ptrdiff_t indexConstraint = constraintsMapping.getValue(constraint.get());
		SURGSIM_ASSERT(indexConstraint >= 0) << "Index for constraint is invalid: " << indexConstraint << std::endl;

		auto contactConstraintData = std::dynamic_pointer_cast<ContactConstraintData>(constraint->getData());
		SURGSIM_ASSERT(contactConstraintData != nullptr) << "";

		auto contact = contactConstraintData->getContact();

		contact->force = lambda[indexConstraint] * contact->normal;
	}

	return result;
}

void SolveMlcp::setMaxIterations(size_t maxIterations)
{
	m_gaussSeidelSolver.setMaxIterations(maxIterations);
}

size_t SolveMlcp::getMaxIterations() const
{
	return m_gaussSeidelSolver.getMaxIterations();
}

void SolveMlcp::setPrecision(double epsilon)
{
	m_gaussSeidelSolver.setEpsilonConvergence(epsilon);
}

double SolveMlcp::getPrecision() const
{
	return m_gaussSeidelSolver.getEpsilonConvergence();
}

void SolveMlcp::setContactTolerance(std::pair<double, double> epsilon)
{
	m_gaussSeidelSolver.setContactTolerance(epsilon);
}

std::pair<double, double> SolveMlcp::getContactTolerance() const
{
	return m_gaussSeidelSolver.getContactTolerance();
}

}; // Physics
}; // SurgSim

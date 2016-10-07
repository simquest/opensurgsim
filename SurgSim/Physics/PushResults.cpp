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
#include "SurgSim/Math/Valid.h"
#include "SurgSim/Physics/ConstraintType.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"
#include "SurgSim/Physics/MlcpPhysicsSolution.h"
#include "SurgSim/Physics/PushResults.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Physics
{

PushResults::PushResults(bool doCopyState) :
	Computation(doCopyState),
	m_logger(SurgSim::Framework::Logger::getLogger("Physics/PushResults"))
{
}

PushResults::~PushResults()
{
}

std::shared_ptr<PhysicsManagerState>
PushResults::doUpdate(const double& dt, const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;
	// 1st step
	// Compute the global dof displacement correction from the constraints forces (result of the MLCP)
	// correction = CHt . lambda
	MlcpPhysicsSolution& solution = result->getMlcpSolution();
	const Eigen::VectorXd& lambda = solution.x;
	if (lambda.size() == 0)
	{
		return state;
	}

	// 2nd step
	// Check for valid Signorini (e.g., good contact tolerances)
	const auto& problem = result->getMlcpProblem();
	bool validSignorini = true;

	if (m_discardBadResults)
	{
		const SurgSim::Math::MlcpProblem::Matrix& A = problem.A;
		const SurgSim::Math::MlcpProblem::Vector& b = problem.b;
		auto& activeConstraints = result->getActiveConstraints();
		auto& constraintsMapping = result->getConstraintsMapping();

		for (const std::shared_ptr<SurgSim::Physics::Constraint>& constraint : activeConstraints)
		{
			if (constraint->getType() == SurgSim::Physics::FRICTIONLESS_3DCONTACT ||
				constraint->getType() == SurgSim::Physics::FRICTIONAL_3DCONTACT)
			{
				const auto index = constraintsMapping.getValue(constraint.get());
				const double violation = b[index] + A.row(index) * lambda;
				// Enforce orthogonality condition
				if (!SurgSim::Math::isValid(violation) || violation < -m_contactTolerance ||
					(lambda[index] > solution.epsilonConvergence &&
					violation > m_contactTolerance))
				{
					validSignorini = false;
					SURGSIM_LOG_INFO(m_logger)
						<< "Invalid contact violation, or violation magnitude greater than contact tolerance.";
					break;
				}
			}
		}
	}

	if (validSignorini)
	{
		// 3rd step
		// Push the dof displacement correction to all representation, using their assigned index
		// Compute the global dof displacement correction from the constraints forces (result of the MLCP)
		const Math::MlcpProblem::Matrix& CHt = problem.CHt;
		Math::MlcpSolution::Vector& dofCorrection = solution.dofCorrection;
		dofCorrection = CHt * lambda;

		SURGSIM_LOG_DEBUG(m_logger) << "b:\t" << problem.b.transpose();
		SURGSIM_LOG_DEBUG(m_logger) << "final:\t" << (problem.A * lambda + problem.b).transpose();
		SURGSIM_LOG_DEBUG(m_logger) << "Lambda:\t" << lambda.transpose();

		auto& representations = result->getActiveRepresentations();
		for (auto& representation : representations)
		{
			if (representation->isActive())
			{
				ptrdiff_t index = result->getRepresentationsMapping().getValue(representation.get());
				SURGSIM_ASSERT(index >= 0) << "Bad index found for representation " << representation->getName()
					<< std::endl;
				representation->applyCorrection(dt, dofCorrection.segment(index, representation->getNumDof()));
			}
		}
	}
	return result;
}

void PushResults::setDiscardBadResults(bool discard)
{
	m_discardBadResults = discard;
}

bool PushResults::isDiscardBadResults() const
{
	return m_discardBadResults;
}

void PushResults::setContactTolerance(double tolerance)
{
	m_contactTolerance = tolerance;
}

double PushResults::getContactTolerance() const
{
	return m_contactTolerance;
}

}; // namespace Physics
}; // namespace SurgSim

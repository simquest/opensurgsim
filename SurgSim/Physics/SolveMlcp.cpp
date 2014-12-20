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

#include <memory>
#include <vector>

#include <Eigen/Core>
using Eigen::MatrixXd;
using Eigen::VectorXd;

#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/SolveMlcp.h"
#include "SurgSim/Math/MlcpGaussSeidelSolver.h"

namespace SurgSim
{
namespace Physics
{

SolveMlcp::SolveMlcp(bool doCopyState) : Computation(doCopyState)
{}

SolveMlcp::~SolveMlcp()
{}

std::shared_ptr<PhysicsManagerState> SolveMlcp::doUpdate(const double& dt,
	const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;

	// Solve the Mlcp using a Gauss-Seidel solver
	m_gaussSeidelSolver.solve(result->getMlcpProblem(), &(result->getMlcpSolution()));

	return result;
}

void SolveMlcp::setMaxIterations(int maxIterations)
{
	m_gaussSeidelSolver.setMaxIterations(maxIterations);
}

int SolveMlcp::getMaxIterations() const
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

void SolveMlcp::setContactTolerance(double epsilon)
{
	m_gaussSeidelSolver.setContactTolerance(epsilon);
}

double SolveMlcp::getContactTolerance() const
{
	return m_gaussSeidelSolver.getContactTolerance();
}

}; // Physics
}; // SurgSim

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

#ifndef MLCP_TEST_PROBLEM_H
#define MLCP_TEST_PROBLEM_H

#include <vector>
#include <Eigen/Core>
#include "MLCP_Constraint.h"


struct MlcpTestData
{
public:
	// Description of an MLCP problem.
	Eigen::MatrixXd HCHt;
	Eigen::VectorXd E;
	std::vector<MLCP_Constraint> constraintTypes;
	Eigen::VectorXd mu;

	// Expected output for the solver (as recorded from old Gauss-Seidel code).
	Eigen::VectorXd expectedLambda;

	// Other metadata
	std::string description;
	std::vector<std::string> flags;
	int numDegreesOfFreedom;

	MlcpTestData() :
		numDegreesOfFreedom(-1)
	{
	}

	MlcpTestData(const MlcpTestData& other) :
		HCHt(other.HCHt), E(other.E), mu(other.mu), constraintTypes(other.constraintTypes),
		expectedLambda(other.expectedLambda),
		description(other.description), flags(other.flags), numDegreesOfFreedom(other.numDegreesOfFreedom)
	{
	}

	MlcpTestData& operator= (const MlcpTestData& other)
	{
		HCHt = other.HCHt;
		E = other.E;
		mu = other.mu;
		constraintTypes = other.constraintTypes;

		expectedLambda = other.expectedLambda;

		description = other.description;
		flags = other.flags;
		numDegreesOfFreedom = other.numDegreesOfFreedom;
	}

	int getSize() const
	{
		return E.rows();
	}
};


const MlcpTestData* getTestProblem1();


#endif // MLCP_TEST_PROBLEM_H

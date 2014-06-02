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

#ifndef SURGSIM_TESTING_MLCPIO_MLCPTESTDATA_H
#define SURGSIM_TESTING_MLCPIO_MLCPTESTDATA_H

#include <memory>
#include <string>

#include <vector>
#include <Eigen/Core>
#include "SurgSim/Math/MlcpProblem.h"
#include "SurgSim/Math/MlcpConstraintType.h"


struct MlcpTestData
{
public:
	// Description of an MLCP problem.
	SurgSim::Math::MlcpProblem problem;

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
		problem(other.problem),
		expectedLambda(other.expectedLambda),
		description(other.description),
		flags(other.flags),
		numDegreesOfFreedom(other.numDegreesOfFreedom)
	{
	}

	MlcpTestData& operator= (const MlcpTestData& other)
	{
		problem = other.problem;
		expectedLambda = other.expectedLambda;

		description = other.description;
		flags = other.flags;
		numDegreesOfFreedom = other.numDegreesOfFreedom;

		return *this;
	}

	ptrdiff_t getSize() const
	{
		return problem.b.rows();
	}
};

const MlcpTestData* getTestProblem1();

std::shared_ptr<MlcpTestData> loadTestData(const std::string& fileName);

std::string getTestFileName(const std::string& prefix, int index, const std::string& suffix);

#endif // SURGSIM_TESTING_MLCPIO_MLCPTESTDATA_H

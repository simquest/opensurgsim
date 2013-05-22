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

#ifndef MLCP_TEST_PROBLEMS_H
#define MLCP_TEST_PROBLEMS_H

#include "TOOLS/Matrix/matrix.h"
#include "TOOLS/Vector/vector.h"
#include "MLCP_Constraint.h"

#include <vector>


struct MlcpTestProblem
{
public:
	//  Description of an MLCP problem.
	Dynamic_Matrix<double> HCHt;
	Dynamic_Vector<double> E;
	Dynamic_Vector<double> mu;
	std::vector<MLCP_Constraint> constraintTypes;

	// Expected output for the solver (as recorded from old Gauss-Seidel code).
	Dynamic_Vector<double> expectedLambda;


	MlcpTestProblem()
	{
	}

	MlcpTestProblem(const MlcpTestProblem& other) :
		HCHt(other.HCHt), E(other.E), mu(other.mu), constraintTypes(other.constraintTypes),
		expectedLambda(other.expectedLambda)
	{
	}

	MlcpTestProblem& operator= (const MlcpTestProblem& other)
	{
		HCHt = other.HCHt;
		E = other.E;
		mu = other.mu;
		constraintTypes = other.constraintTypes;
		expectedLambda = other.expectedLambda;
	}

	int getSize() const
	{
		return E.getSize();
	}
};


const MlcpTestProblem* getTestProblem1();


#endif // MLCP_TEST_PROBLEMS_H

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

/// \file SolveMlcpTests.cpp
/// Simple Test for SolveMlcp computation

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/SolveMlcp.h"

#include "SurgSim/Testing/MlcpIO/MlcpTestData.h"
#include "SurgSim/Testing/MlcpIO/ReadText.h"

using SurgSim::Physics::PhysicsManagerState;
using SurgSim::Physics::SolveMlcp;

namespace
{
	const double epsilon = 1e-10;
};

TEST(SolveMlcpTest, CanConstruct)
{
	ASSERT_NO_THROW({std::shared_ptr<SolveMlcp> solveMlcpComputation = std::make_shared<SolveMlcp>();});
}

static void testMlcp(const std::string& filename, double contactTolerance, double solverPrecision, size_t maxIteration)
{
	std::shared_ptr<MlcpTestData> data = loadTestData(filename);
	ASSERT_NE(nullptr, data) << "Could not load data file 'mlcpOriginalTest.txt'";

	std::shared_ptr<PhysicsManagerState> state = std::make_shared<PhysicsManagerState>();
	// default parameter explicitly marked just to be clear, we are working in 1 instance of the state, no copy.
	std::shared_ptr<SolveMlcp> solveMlcpComputation = std::make_shared<SolveMlcp>(false);
	double dt = 1e-3;

	solveMlcpComputation->setContactTolerance(contactTolerance);
	EXPECT_NEAR(contactTolerance, solveMlcpComputation->getContactTolerance(), 1e-10);
	solveMlcpComputation->setPrecision(solverPrecision);
	EXPECT_NEAR(solverPrecision, solveMlcpComputation->getPrecision(), 1e-10);
	solveMlcpComputation->setMaxIterations(maxIteration);
	EXPECT_EQ(maxIteration, solveMlcpComputation->getMaxIterations());

	// Copy the MlcpProblem data over into the input state
	state->getMlcpProblem().A = data->problem.A;
	state->getMlcpProblem().b = data->problem.b;
	state->getMlcpProblem().constraintTypes = data->problem.constraintTypes;
	state->getMlcpProblem().mu = data->problem.mu;

	// Allocate the MlcpSolution to store the output
	state->getMlcpSolution().x.resize(state->getMlcpProblem().b.size());
	state->getMlcpSolution().x.setZero();

	// Solve the mlcp problem and store the solution in the output state
	state = solveMlcpComputation->update(dt, state);

	// Compare the mlcp solution with the expected one
	EXPECT_TRUE(state->getMlcpSolution().x.isApprox(data->expectedLambda, epsilon)) <<
		"lambda:" << std::endl << state->getMlcpSolution().x.transpose() << std::endl <<
		"expected:" << std::endl << data->expectedLambda.transpose() << std::endl;
}

TEST(SolveMlcpTest, TestOriginalMlcp)
{
	testMlcp("mlcpOriginalTest.txt", 2e-4, 1e-4, 30);
}

TEST(SolveMlcpTest, TestSequenceMlcps)
{
	for (int i = 0;  i <= 9;  ++i)
	{
		std::ostringstream scopeName;
		scopeName << "Testing Mlcp " << i;
		SCOPED_TRACE(scopeName.str());

		testMlcp(getTestFileName("mlcpTest", i, ".txt"), 1e-9, 1e-9, 100);
	}
}

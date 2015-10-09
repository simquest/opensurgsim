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

/// \file
/// Tests for the class OdeSolver.
///

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/UnitTests/MockObject.h"

namespace SurgSim
{

namespace Math
{

namespace
{
const std::string name = "MockOdeSolver";
};

class MockOdeSolver : public OdeSolver
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit MockOdeSolver(OdeEquation* equation) : OdeSolver(equation)
	{
		this->m_name = name;
	}

	/// Virtual destructor
	virtual ~MockOdeSolver()
	{
	}

	void solve(double dt, const OdeState& currentState, OdeState* newState, bool computeCompliance = true) override
	{
	}

	void assembleLinearSystem(double dt, const OdeState& state, const OdeState& newState, bool computeRHS) override
	{

	}
};

TEST(OdeSolver, ConstructorTest)
{
	// OdeEquation is tested separately and is considered valid to use here.
	MassPoint m;
	std::shared_ptr<MockOdeSolver> solver;

	ASSERT_NO_THROW({solver = std::make_shared<MockOdeSolver>(&m); });
	EXPECT_NE(nullptr, solver->getLinearSolver());
	EXPECT_NE(nullptr, std::dynamic_pointer_cast<LinearSparseSolveAndInverseLU>(solver->getLinearSolver()));

}

TEST(OdeSolver, GetTest)
{
	MassPoint m;
	MockOdeSolver solver(&m);

	EXPECT_EQ(name, solver.getName());

	EXPECT_NE(nullptr, solver.getLinearSolver());
	EXPECT_NE(nullptr, std::dynamic_pointer_cast<LinearSparseSolveAndInverseLU>(solver.getLinearSolver()));
	EXPECT_NO_THROW(solver.setLinearSolver(std::make_shared<LinearSparseSolveAndInverseLU>()));
	EXPECT_NE(nullptr, solver.getLinearSolver());
	EXPECT_NE(nullptr, std::dynamic_pointer_cast<LinearSparseSolveAndInverseLU>(solver.getLinearSolver()));
}

}; // namespace Math

}; // namespace SurgSim

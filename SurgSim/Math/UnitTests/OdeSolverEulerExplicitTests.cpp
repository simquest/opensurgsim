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
/// Tests for the class OdeSolverEulerExplicit.
///

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Math/OdeSolverEulerExplicit.h"
#include "SurgSim/Math/OdeSolverLinearEulerExplicit.h"
#include "SurgSim/Math/UnitTests/MockObject.h"

namespace SurgSim
{

namespace Math
{

namespace
{
template<class T>
void doConstructorTest()
{
	MassPoint m;
	ASSERT_NO_THROW({auto solver = std::make_shared<T>(&m); m.setOdeSolver(solver);});
}
};

TEST(OdeSolverEulerExplicit, ConstructorTest)
{
	{
		SCOPED_TRACE("EulerExplicit");
		doConstructorTest<OdeSolverEulerExplicit>();
	}
	{
		SCOPED_TRACE("LinearEulerExplicit");
		doConstructorTest<OdeSolverLinearEulerExplicit>();
	}
}

namespace
{
template<class T>
void doSolveTest()
{
	// Test 2 iterations because Linear solvers have a different algorithm on the 1st pass from the following passes.

	{
		MassPoint m;
		MassPointState defaultState, state0, state1, state2;
		auto solver = std::make_shared<T>(&m);
		m.setOdeSolver(solver);

		// ma = mg <=> a = g
		// v(1) = g.dt + v(0)
		// x(1) = v(0).dt + x(0)
		ASSERT_NO_THROW({solver->solve(1e-3, state0, &state1);});
		EXPECT_EQ(defaultState, state0);
		EXPECT_NE(defaultState, state1);
		EXPECT_TRUE(state1.getVelocities().isApprox(m.m_gravity * 1e-3 + state0.getVelocities()));
		EXPECT_TRUE(state1.getPositions().isApprox(state0.getVelocities() * 1e-3 + state0.getPositions()));

		// v(2) = g.dt + v(1)
		// x(2) = v(1).dt + x(1)
		ASSERT_NO_THROW({solver->solve(1e-3, state1, &state2);});
		EXPECT_NE(defaultState, state1);
		EXPECT_NE(defaultState, state2);
		EXPECT_NE(state2, state1);
		EXPECT_TRUE(state2.getVelocities().isApprox(m.m_gravity * 1e-3 + state1.getVelocities()));
		EXPECT_TRUE(state2.getPositions().isApprox(state1.getVelocities() * 1e-3 + state1.getPositions()));
	}

	{
		MassPoint m(0.1);
		MassPointState defaultState, state0, state1, state2;
		auto solver = std::make_shared<T>(&m);
		m.setOdeSolver(solver);

		// ma = mg - c.v <=> a = g - c/m.v
		// v(1) = (g - c/m.v).dt + v(0)
		// x(1) = v(0).dt + x(0)
		ASSERT_NO_THROW({solver->solve(1e-3, state0, &state1);});
		EXPECT_EQ(defaultState, state0);
		EXPECT_NE(defaultState, state1);
		Vector3d acceleration0 = m.m_gravity - 0.1 * state0.getVelocities() / m.m_mass;
		EXPECT_TRUE(state1.getVelocities().isApprox(acceleration0 * 1e-3 + state0.getVelocities()));
		EXPECT_TRUE(state1.getPositions().isApprox(state0.getVelocities() * 1e-3  + state0.getPositions()));

		// v(2) = (g - c/m.v).dt + v(1)
		// x(2) = v(1).dt + x(1)
		ASSERT_NO_THROW({solver->solve(1e-3, state1, &state2);});
		EXPECT_NE(defaultState, state1);
		EXPECT_NE(defaultState, state2);
		EXPECT_NE(state1, state2);
		Vector3d acceleration1 = m.m_gravity - 0.1 * state1.getVelocities() / m.m_mass;
		EXPECT_TRUE(state2.getVelocities().isApprox(acceleration1 * 1e-3 + state1.getVelocities()));
		EXPECT_TRUE(state2.getPositions().isApprox(state1.getVelocities() * 1e-3 + state1.getPositions()));
	}
}
};

TEST(OdeSolverEulerExplicit, SolveTest)
{
	{
		SCOPED_TRACE("EulerExplicit solve test");
		doSolveTest<OdeSolverEulerExplicit>();
	}
	{
		SCOPED_TRACE("LinearEulerExplicit solve test");
		doSolveTest<OdeSolverLinearEulerExplicit>();
	}
}

namespace
{
template <class T>
void doComputeMatricesTest()
{
	MassPoint m;
	auto solver = std::make_shared<T>(&m);
	m.setOdeSolver(solver);
	MassPointState state;
	double dt = 1e-3;

	Matrix expectedSystemMatrix = m.computeM(state) / dt;
	EXPECT_NO_THROW(solver->computeMatrices(dt, state));
	EXPECT_TRUE(solver->getSystemMatrix().isApprox(expectedSystemMatrix));
	EXPECT_TRUE(m.applyCompliance(state, Matrix::Identity(state.getNumDof(),
								  state.getNumDof())).isApprox(expectedSystemMatrix.inverse()));
}
};

TEST(OdeSolverEulerExplicit, ComputeMatricesTest)
{
	{
		SCOPED_TRACE("EulerExplicit");
		doComputeMatricesTest<OdeSolverEulerExplicit>();
	}

	{
		SCOPED_TRACE("LinearEulerExplicit");
		doComputeMatricesTest<OdeSolverLinearEulerExplicit>();
	}
}

}; // Math

}; // SurgSim

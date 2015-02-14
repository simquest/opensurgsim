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
/// Tests for the class OdeSolverEulerImplicit.
///

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Math/OdeSolverEulerImplicit.h"
#include "SurgSim/Math/OdeSolverLinearEulerImplicit.h"
#include "SurgSim/Math/UnitTests/MockObject.h"

namespace SurgSim
{

namespace Math
{

template<class T>
void doConstructorTest()
{
	MassPoint m;
	ASSERT_NO_THROW({T solver(&m);});
}

TEST(OdeSolverEulerImplicit, ConstructorTest)
{
	{
		SCOPED_TRACE("EulerImplicit");
		doConstructorTest<OdeSolverEulerImplicit>();
	}
	{
		SCOPED_TRACE("LinearEulerImplicit");
		doConstructorTest<OdeSolverLinearEulerImplicit>();
	}
}

template<class T>
void doSolveTest()
{
	// Test 2 iterations because Linear solvers have a different algorithm on the 1st pass from the following passes.

	{
		MassPoint m;
		MassPointState defaultState, state0, state1, state2;
		T solver(&m);
		solver.setNewtonRaphsonMaximumIteration(1);

		// ma = mg <=> a = g
		// v(1) = g.dt + v(0)
		// x(1) = v(1).dt + x(0)
		ASSERT_NO_THROW({solver.solve(1e-3, state0, &state1);});
		EXPECT_EQ(defaultState, state0);
		EXPECT_NE(defaultState, state1);
		EXPECT_TRUE(state1.getVelocities().isApprox(m.m_gravity * 1e-3 + state0.getVelocities()));
		EXPECT_TRUE(state1.getPositions().isApprox(state1.getVelocities() * 1e-3 + state0.getPositions()));

		// v(2) = g.dt + v(1)
		// x(2) = v(2).dt + x(1)
		ASSERT_NO_THROW({solver.solve(1e-3, state1, &state2);});
		EXPECT_NE(defaultState, state1);
		EXPECT_NE(defaultState, state2);
		EXPECT_NE(state2, state1);
		EXPECT_TRUE(state2.getVelocities().isApprox(m.m_gravity * 1e-3 + state1.getVelocities()));
		EXPECT_TRUE(state2.getPositions().isApprox(state2.getVelocities() * 1e-3 + state1.getPositions()));
	}

	{
		MassPoint m(0.1);
		MassPointState defaultState, state0, state1, state2;
		T solver(&m);
		solver.setNewtonRaphsonMaximumIteration(1);

		// ma = mg - c.v <=> a = g - c/m.v
		// v(1) = (g - c/m.v(1)).dt + v(0) <=> v(1) = I.(1.0 + dt.c/m)^-1.(g.dt + v(0))
		// x(1) = v(1).dt + x(0)
		ASSERT_NO_THROW({solver.solve(1e-3, state0, &state1);});
		EXPECT_EQ(defaultState, state0);
		EXPECT_NE(defaultState, state1);
		Matrix33d systemInverse = Matrix33d::Identity() * 1.0 / (1.0 + 1e-3 * 0.1 / m.m_mass);
		EXPECT_TRUE(state1.getVelocities().isApprox(systemInverse * (m.m_gravity * 1e-3 + state0.getVelocities())));
		EXPECT_TRUE(state1.getPositions().isApprox(state1.getVelocities() * 1e-3  + state0.getPositions()));

		// v(2) = (g - c/m.v(2)).dt + v(1) <=> v(2) = I.(1.0 + dt.c/m)^-1.(g.dt + v(1))
		// x(2) = v(2).dt + x(1)
		ASSERT_NO_THROW({solver.solve(1e-3, state1, &state2);});
		EXPECT_NE(defaultState, state1);
		EXPECT_NE(defaultState, state2);
		EXPECT_NE(state1, state2);
		EXPECT_TRUE(state2.getVelocities().isApprox(systemInverse * (m.m_gravity * 1e-3 + state1.getVelocities())));
		EXPECT_TRUE(state2.getPositions().isApprox(state2.getVelocities() * 1e-3 + state1.getPositions()));
	}
}

TEST(OdeSolverEulerImplicit, SolveTest)
{
	{
		SCOPED_TRACE("EulerImplicit");
		doSolveTest<OdeSolverEulerImplicit>();
	}
	{
		SCOPED_TRACE("LinearEulerImplicit");
		doSolveTest<OdeSolverLinearEulerImplicit>();
	}
}

}; // Math

}; // SurgSim

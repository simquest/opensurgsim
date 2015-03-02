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
/// Tests for the class OdeSolverEulerExplicitModified.
///

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Math/OdeSolverEulerExplicitModified.h"
#include "SurgSim/Math/OdeSolverLinearEulerExplicitModified.h"
#include "SurgSim/Math/UnitTests/MockObject.h"

namespace SurgSim
{

namespace Math
{

namespace anonymous
{
template<class T>
void doConstructorTest()
{
	MassPoint m;
	ASSERT_NO_THROW({T solver(&m);});
}
};

TEST(OdeSolverEulerExplicitModified, ConstructorTest)
{
	{
		SCOPED_TRACE("EulerExplicitModified");
		anonymous::doConstructorTest<OdeSolverEulerExplicitModified>();
	}
	{
		SCOPED_TRACE("LinearEulerExplicitModified");
		anonymous::doConstructorTest<OdeSolverLinearEulerExplicitModified>();
	}
}

namespace anonymous
{
template<class T>
void doSolveTest(bool computeCompliance)
{
	// Test 2 iterations because Linear solvers have a different algorithm on the 1st pass from the following passes.

	{
		MassPoint m;
		MassPointState defaultState, state0, state1, state2;
		T solver(&m);

		// ma = mg <=> a = g
		// v(1) = g.dt + v(0)
		// x(1) = v(1).dt + x(0)
		ASSERT_NO_THROW({solver.solve(1e-3, state0, &state1, computeCompliance);});
		EXPECT_EQ(defaultState, state0);
		EXPECT_NE(defaultState, state1);
		EXPECT_TRUE(state1.getVelocities().isApprox(m.m_gravity * 1e-3 + state0.getVelocities()));
		EXPECT_TRUE(state1.getPositions().isApprox(state1.getVelocities() * 1e-3 + state0.getPositions()));

		// v(2) = g.dt + v(1)
		// x(2) = v(2).dt + x(1)
		ASSERT_NO_THROW({solver.solve(1e-3, state1, &state2, computeCompliance);});
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

		// ma = mg - c.v <=> a = g - c/m.v
		// v(1) = (g - c/m.v).dt + v(0)
		// x(1) = v(1).dt + x(0)
		ASSERT_NO_THROW({solver.solve(1e-3, state0, &state1, computeCompliance);});
		EXPECT_EQ(defaultState, state0);
		EXPECT_NE(defaultState, state1);
		Vector3d acceleration0 = m.m_gravity - 0.1 * state0.getVelocities() / m.m_mass;
		EXPECT_TRUE(state1.getVelocities().isApprox(acceleration0 * 1e-3 + state0.getVelocities()));
		EXPECT_TRUE(state1.getPositions().isApprox(state1.getVelocities() * 1e-3  + state0.getPositions()));

		// v(2) = (g - c/m.v).dt + v(1)
		// x(2) = v(2).dt + x(1)
		ASSERT_NO_THROW({solver.solve(1e-3, state1, &state2, computeCompliance);});
		EXPECT_NE(defaultState, state1);
		EXPECT_NE(defaultState, state2);
		EXPECT_NE(state1, state2);
		Vector3d acceleration1 = m.m_gravity - 0.1 * state1.getVelocities() / m.m_mass;
		EXPECT_TRUE(state2.getVelocities().isApprox(acceleration1 * 1e-3 + state1.getVelocities()));
		EXPECT_TRUE(state2.getPositions().isApprox(state2.getVelocities() * 1e-3 + state1.getPositions()));
	}
}
};

TEST(OdeSolverEulerExplicitModified, SolveTest)
{
	{
		SCOPED_TRACE("EulerExplicitModified computing the compliance matrix");
		anonymous::doSolveTest<OdeSolverEulerExplicitModified>(true);
	}
	{
		SCOPED_TRACE("EulerExplicitModified not computing the compliance matrix");
		anonymous::doSolveTest<OdeSolverEulerExplicitModified>(false);
	}

	{
		SCOPED_TRACE("LinearEulerExplicitModified computing the compliance matrix");
		anonymous::doSolveTest<OdeSolverLinearEulerExplicitModified>(true);
	}
	{
		SCOPED_TRACE("LinearEulerExplicitModified not computing the compliance matrix");
		anonymous::doSolveTest<OdeSolverLinearEulerExplicitModified>(false);
	}
}

namespace anonymous
{
template <class T>
void doComputeMatricesTest()
{
	MassPoint m;
	T solver(&m);
	MassPointState state;
	double dt = 1e-3;

	Matrix expectedSystemMatrix = m.computeM(state) / dt;
	EXPECT_NO_THROW(solver.computeMatrices(dt, state));
	EXPECT_TRUE(solver.getSystemMatrix().isApprox(expectedSystemMatrix));
	EXPECT_TRUE(solver.getComplianceMatrix().isApprox(expectedSystemMatrix.inverse()));
}
};

TEST(OdeSolverEulerExplicitModified, ComputeMatricesTest)
{
	{
		SCOPED_TRACE("EulerExplicitModified");
		anonymous::doComputeMatricesTest<OdeSolverEulerExplicitModified>();
	}

	{
		SCOPED_TRACE("LinearEulerExplicitModified");
		anonymous::doComputeMatricesTest<OdeSolverLinearEulerExplicitModified>();
	}
}

}; // Math

}; // SurgSim

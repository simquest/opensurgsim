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
/// Tests for the class OdeSolverStatic.
///

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Math/OdeSolverStatic.h"
#include "SurgSim/Math/OdeSolverLinearStatic.h"
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
	MassPointsForStatic m;
	ASSERT_NO_THROW({T solver(&m);});
}
};
TEST(OdeSolverStatic, ConstructorTest)
{
	{
		SCOPED_TRACE("Static");
		doConstructorTest<OdeSolverStatic>();
	}
	{
		SCOPED_TRACE("LinearStatic");
		doConstructorTest<OdeSolverLinearStatic>();
	}
}

namespace
{
template<class T>
void doSolveTest(bool computeCompliance)
{
	MassPointsForStatic m;
	MassPointsStateForStatic defaultState, currentState, newState;

	T solver(&m);
	ASSERT_NO_THROW({solver.solve(1e-3, currentState, &newState, computeCompliance);});
	EXPECT_EQ(defaultState, currentState);
	EXPECT_NE(defaultState, newState);

	// Solve manually K.(x - x0) = Fext
	const Vector &Fext = m.getExternalForces();
	Vector expectedDeltaX = m.computeK(currentState).inverse() * Fext;
	Vector expectedX = defaultState.getPositions() + expectedDeltaX;
	EXPECT_TRUE(newState.getPositions().isApprox(expectedX));
}
};

TEST(OdeSolverStatic, SolveTest)
{
	{
		SCOPED_TRACE("Static computing the compliance matrix");
		doSolveTest<OdeSolverStatic>(true);
	}
	{
		SCOPED_TRACE("Static not computing the compliance matrix");
		doSolveTest<OdeSolverStatic>(false);
	}

	{
		SCOPED_TRACE("LinearStatic computing the compliance matrix");
		doSolveTest<OdeSolverLinearStatic>(true);
	}
	{
		SCOPED_TRACE("LinearStatic not computing the compliance matrix");
		doSolveTest<OdeSolverLinearStatic>(false);
	}
}

namespace
{
template <class T>
void doComputeMatricesTest()
{
	MassPointsForStatic m;
	T solver(&m);
	MassPointsStateForStatic state;
	double dt = 1e-3;

	Matrix expectedSystemMatrix = m.computeK(state);
	EXPECT_NO_THROW(solver.computeMatrices(dt, state));
	EXPECT_TRUE(solver.getSystemMatrix().isApprox(expectedSystemMatrix));
	EXPECT_TRUE(solver.getComplianceMatrix().isApprox(expectedSystemMatrix.inverse()));
}
};

TEST(OdeSolverStatic, ComputeMatricesTest)
{
	{
		SCOPED_TRACE("Static");
		doComputeMatricesTest<OdeSolverStatic>();
	}

	{
		SCOPED_TRACE("LinearStatic");
		doComputeMatricesTest<OdeSolverLinearStatic>();
	}
}

}; // Math

}; // SurgSim

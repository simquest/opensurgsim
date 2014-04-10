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

template<class T>
void doConstructorTest()
{
	MassPointsForStatic m;
	ASSERT_NO_THROW({T solver(&m);});
}

TEST(OdeSolverStatic, ConstructorTest)
{
	{
		SCOPED_TRACE("Static");
		doConstructorTest<Static<MassPointsStateForStatic, Matrix, Matrix, Matrix, Matrix>>();
	}
	{
		SCOPED_TRACE("LinearStatic");
		doConstructorTest<LinearStatic<MassPointsStateForStatic, Matrix, Matrix, Matrix, Matrix>>();
	}
}

template<class T>
void doSolveTest()
{
	MassPointsForStatic m;
	MassPointsStateForStatic defaultState, currentState, newState;

	T solver(&m);
	ASSERT_NO_THROW({solver.solve(1e-3, currentState, &newState);});
	EXPECT_EQ(defaultState, currentState);
	EXPECT_NE(defaultState, newState);

	// Solve manually K.(x - x0) = Fext
	const Vector &Fext = m.getExternalForces();
	Vector expectedU = m.computeK(currentState).inverse() * Fext;
	Vector expectedX = defaultState.getPositions() + expectedU;
	EXPECT_TRUE(newState.getPositions().isApprox(expectedX));
}

TEST(OdeSolverStatic, SolveTest)
{
	{
		SCOPED_TRACE("Static");
		doSolveTest<Static<MassPointsStateForStatic, Matrix, Matrix, Matrix, Matrix>>();
	}
	{
		SCOPED_TRACE("LinearStatic");
		doSolveTest<LinearStatic<MassPointsStateForStatic, Matrix, Matrix, Matrix, Matrix>>();
	}
}

}; // Math

}; // SurgSim

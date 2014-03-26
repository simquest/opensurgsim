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

template<class T>
void doConstructorTest()
{
	MassPoint m;

	ASSERT_NO_THROW({T solver(&m);});
	ASSERT_NO_THROW({T* solver = new T(&m); delete solver;});
	ASSERT_NO_THROW({std::shared_ptr<T> solver = std::make_shared<T>(&m);});
}

TEST(OdeSolverEulerExplicitModified, ConstructorTest)
{
	{
		SCOPED_TRACE("EulerExplicitModified");
		doConstructorTest<ModifiedExplicitEuler<MassPointState, Matrix, Matrix, Matrix, Matrix>>();
	}
	{
		SCOPED_TRACE("LinearEulerExplicitModified");
		doConstructorTest<LinearModifiedExplicitEuler<MassPointState, Matrix, Matrix, Matrix, Matrix>>();
	}
}

template<class T>
void doSolveTest()
{
	{
		MassPoint m;
		MassPointState defaultState, currentState, newState;

		T solver(&m);
		ASSERT_NO_THROW({solver.solve(1e-3, currentState, &newState);});
		EXPECT_EQ(defaultState, currentState);
		EXPECT_NE(defaultState, newState);
		EXPECT_TRUE(newState.getVelocities().isApprox(m.m_gravity * 1e-3));
		EXPECT_TRUE(newState.getPositions().isApprox(m.m_gravity * 1e-3 * 1e-3));
	}

	{
		MassPoint m(0.1);
		MassPointState defaultState, currentState, newState;

		T solver(&m);
		ASSERT_NO_THROW({solver.solve(1e-3, currentState, &newState);});
		EXPECT_EQ(defaultState, currentState);
		EXPECT_NE(defaultState, newState);
		EXPECT_TRUE(newState.getVelocities().isApprox((m.m_gravity - 0.1 * currentState.getVelocities()) * 1e-3));
		EXPECT_TRUE(newState.getPositions().isApprox((m.m_gravity - 0.1 * currentState.getVelocities()) * 1e-3 * 1e-3));
	}
}

TEST(OdeSolverEulerExplicitModified, SolveTest)
{
	{
		SCOPED_TRACE("EulerExplicitModified");
		doSolveTest<ModifiedExplicitEuler<MassPointState, Matrix, Matrix, Matrix, Matrix>>();
	}
	{
		SCOPED_TRACE("LinearEulerExplicitModified");
		doSolveTest<LinearModifiedExplicitEuler<MassPointState, Matrix, Matrix, Matrix, Matrix>>();
	}
}

}; // Math

}; // SurgSim

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
#include "SurgSim/Math/UnitTests/MockObject.h"

namespace SurgSim
{

namespace Math
{

TEST(OdeSolverEulerImplicit, ConstructorTest)
{
	typedef ImplicitEuler<MassPointState, Matrix, Matrix, Matrix, Matrix> SolverType;

	MassPoint m;

	ASSERT_NO_THROW({SolverType solver(&m);});
	ASSERT_NO_THROW({SolverType* solver = new SolverType(&m); delete solver;});
	ASSERT_NO_THROW({std::shared_ptr<SolverType> solver = std::make_shared<SolverType>(&m);});
}

TEST(OdeSolverEulerImplicit, SolveTest)
{
	typedef ImplicitEuler<MassPointState, Matrix, Matrix, Matrix, Matrix> SolverType;

	{
		SCOPED_TRACE("OdeSolverEulerImplicit solve test with 0 viscosity");
		MassPoint m;
		MassPointState defaultState, currentState, newState;

		SolverType solver(&m);
		ASSERT_NO_THROW({solver.solve(1e-3, currentState, &newState);});
		EXPECT_EQ(defaultState, currentState);
		EXPECT_NE(defaultState, newState);
		EXPECT_TRUE(newState.getVelocities().isApprox(m.m_gravity * 1e-3));
		EXPECT_TRUE(newState.getPositions().isApprox(m.m_gravity * 1e-3 * 1e-3));
	}
	{
		// Ma(t+dt) = F(t+dt)
		// ma(t+dt) = mg - cv(t+dt)
		// ma(t+dt) + cv(t+dt) = mg
		// using the Backward Euler integration scheme:
		// m(v(t+dt) - v(t))/dt + cv(t+dt) = mg
		// v(t+dt) (m/dt + c) = mg + mv(t)/dt
		// v(t+dt) = [ mg + mv(t)/dt ] / (m/dt + c) = [g + v(t)/dt] . m/(m/dt + c)
		// using the Backward Euler integration scheme to compute the new position:
		// x(t+dt) = x(t) + dt. v(t+dt)
		SCOPED_TRACE("OdeSolverEulerImplicit solve test with non 0 viscosity");
		const double dt = 1e-3;
		const double viscosity = 0.1;
		MassPoint m(viscosity);
		MassPointState defaultState, currentState, newState;

		double coef = m.m_mass / (m.m_mass/dt + viscosity);

		SolverType solver(&m);
		ASSERT_NO_THROW({solver.solve(dt, currentState, &newState);});
		EXPECT_EQ(defaultState, currentState);
		EXPECT_NE(defaultState, newState);
		Vector3d newVelocity = (m.m_gravity + currentState.getVelocities()/dt) * coef;
		EXPECT_TRUE(newState.getVelocities().isApprox(newVelocity));
		EXPECT_TRUE(newState.getPositions().isApprox(currentState.getPositions() + dt * newVelocity));
	}
}

}; // Math

}; // SurgSim

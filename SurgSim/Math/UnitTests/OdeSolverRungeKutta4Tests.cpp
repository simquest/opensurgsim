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

/// \file OdeSolverRungeKutta4Tests.cpp
/// Tests for the class OdeSolverRungeKutta4.

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Math/OdeSolverRungeKutta4.h"
#include "SurgSim/Math/OdeSolverLinearRungeKutta4.h"
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
	ASSERT_NO_THROW({T solver(&m);});
}
};

TEST(OdeSolverRungeKutta4, ConstructorTest)
{
	{
		SCOPED_TRACE("OdeSolverRungeKutta4");
		doConstructorTest<OdeSolverRungeKutta4>();
	}
	{
		SCOPED_TRACE("OdeSolverLinearRungeKutta4");
		doConstructorTest<OdeSolverLinearRungeKutta4>();
	}
}

namespace
{
struct RungeKuttaState
{
	RungeKuttaState() {}
	RungeKuttaState(const Vector& p, const Vector& v) : position(p), velocity(v) {}
	Vector position;
	Vector velocity;
};

struct RungeKuttaDerivedState
{
	RungeKuttaDerivedState() {}
	RungeKuttaDerivedState(const Vector& v, const Vector& a) : velocity(v), acceleration(a) {}
	Vector velocity;
	Vector acceleration;
};

void integrateRK4(double dt, const MassPoint& m, const RungeKuttaState& yn, RungeKuttaState* yn_plus_1)
{
	RungeKuttaDerivedState k1, k2, k3, k4;

	// Problem to solve is
	// m.a = m.g - c.v which is an ode of order 2 that can be reduced to order 1 as following:
	// y' = (x)' = (  v  ) = f(t, y)
	//      (v)  = (m.g/m - c.v/m)
	// In terms of (x), f(t, (x)) = (v)
	//             (v)       (v)    (g - c.v/m)

	// Runge Kutta 4 computes y(n+1) = y(n) + 1/6.dt.(k1 + 2 * k2 + 2 * k3 + k4)
	// with k1 = f(t(n)       , y(n)            )
	// with k2 = f(t(n) + dt/2, y(n) + k1 * dt/2)
	// with k3 = f(t(n) + dt/2, y(n) + k2 * dt/2)
	// with k4 = f(t(n) + dt  , y(n) + k3 * dt  )

	// 1st evaluation k1 = f(t(n)       , y(n)            )
	k1.velocity = yn.velocity;
	k1.acceleration = m.m_gravity - m.m_viscosity * yn.velocity / m.m_mass;

	// 2nd evaluation k2 = f(t(n) + dt/2, y(n) + k1 * dt/2)
	k2.velocity = yn.velocity + k1.acceleration * dt / 2.0;
	k2.acceleration = m.m_gravity - m.m_viscosity * (yn.velocity + k1.acceleration * dt / 2.0) / m.m_mass;

	// 3rd evaluation k3 = f(t(n) + dt/2, y(n) + k2 * dt/2)
	k3.velocity = yn.velocity + k2.acceleration * dt / 2.0;
	k3.acceleration = m.m_gravity - m.m_viscosity * (yn.velocity + k2.acceleration * dt / 2.0) / m.m_mass;

	// 4th evaluation k4 = f(t(n) + dt  , y(n) + k3 * dt  )
	k4.velocity = yn.velocity + k3.acceleration * dt;
	k4.acceleration = m.m_gravity - m.m_viscosity * (yn.velocity + k3.acceleration * dt) / m.m_mass;

	yn_plus_1->position = yn.position + dt / 6.0 *
						  (k1.velocity + k4.velocity + 2.0 * (k2.velocity + k3.velocity));
	yn_plus_1->velocity = yn.velocity + dt / 6.0 *
						  (k1.acceleration + k4.acceleration + 2.0 * (k2.acceleration + k3.acceleration));
}

template<class T>
void doSolveTest()
{
	Vector deltaWithoutViscosity;
	Vector deltaWithViscosity;
	double dt = 1e-3;

	// Test direction correctness of a moving point under gravity (no viscosity)
	{
		MassPoint m;
		MassPointState defaultState, currentState, newState;
		defaultState.getVelocities().setZero();
		defaultState.getPositions().setZero();
		currentState = defaultState;
		newState = defaultState;

		auto solver = std::make_shared<T>(&m);
		m.setOdeSolver(solver);
		ASSERT_NO_THROW({solver->solve(dt, currentState, &newState);});
		EXPECT_EQ(defaultState, currentState);
		EXPECT_NE(defaultState, newState);

		EXPECT_FALSE(newState.getVelocities().isZero());
		EXPECT_DOUBLE_EQ(0.0, newState.getVelocities().dot(Vector3d::UnitX()));
		EXPECT_GT(0.0, newState.getVelocities().dot(Vector3d::UnitY()));
		EXPECT_DOUBLE_EQ(0.0, newState.getVelocities().dot(Vector3d::UnitZ()));

		EXPECT_FALSE(newState.getPositions().isZero());
		deltaWithoutViscosity = (newState.getPositions() - currentState.getPositions());
		EXPECT_DOUBLE_EQ(0.0, deltaWithoutViscosity.dot(Vector3d::UnitX()));
		EXPECT_GT(0.0, deltaWithoutViscosity.dot(Vector3d::UnitY()));
		EXPECT_DOUBLE_EQ(0.0, deltaWithoutViscosity.dot(Vector3d::UnitZ()));
	}

	// Test direction correctness of a moving point under gravity (viscosity)
	{
		MassPoint m(0.1);
		MassPointState defaultState, currentState, newState;
		defaultState.getVelocities().setZero();
		defaultState.getPositions().setZero();
		currentState = defaultState;
		newState = defaultState;

		auto solver = std::make_shared<T>(&m);
		m.setOdeSolver(solver);
		ASSERT_NO_THROW({solver->solve(dt, currentState, &newState);});
		EXPECT_EQ(defaultState, currentState);
		EXPECT_NE(defaultState, newState);

		EXPECT_FALSE(newState.getVelocities().isZero());
		EXPECT_FALSE(newState.getVelocities().isApprox(currentState.getVelocities()));
		EXPECT_DOUBLE_EQ(currentState.getVelocities()[0], newState.getVelocities()[0]);
		EXPECT_NE(currentState.getVelocities()[1], newState.getVelocities()[1]);
		EXPECT_LT(newState.getVelocities()[1], currentState.getVelocities()[1]);
		EXPECT_DOUBLE_EQ(currentState.getVelocities()[2], newState.getVelocities()[2]);

		EXPECT_FALSE(newState.getPositions().isZero());
		deltaWithViscosity = (newState.getPositions() - currentState.getPositions());
		EXPECT_DOUBLE_EQ(0.0, deltaWithViscosity.dot(Vector3d::UnitX()));
		EXPECT_GT(0.0, deltaWithViscosity.dot(Vector3d::UnitY()));
		EXPECT_DOUBLE_EQ(0.0, deltaWithViscosity.dot(Vector3d::UnitZ()));
	}

	EXPECT_GT(deltaWithoutViscosity.norm(), deltaWithViscosity.norm());

	// Test Runge Kutta 4 algorithm itself (without viscosity)
	// Test 2 iterations because Linear solvers have a different algorithm on the 1st pass from the following passes.
	{
		SCOPED_TRACE("RK4 without viscosity");

		MassPoint m(0.0);
		MassPointState currentState, newState, newState2;
		currentState.getPositions().setLinSpaced(1.0, 3.0);
		currentState.getVelocities().setConstant(1.0);

		auto solver = std::make_shared<T>(&m);
		m.setOdeSolver(solver);

		RungeKuttaState yn(currentState.getPositions(), currentState.getVelocities());
		RungeKuttaState yn_plus_1, yn_plus_2;

		EXPECT_TRUE(currentState.getPositions().isApprox(yn.position));
		EXPECT_TRUE(currentState.getVelocities().isApprox(yn.velocity));

		// 1st time step
		integrateRK4(dt, m, yn, &yn_plus_1);
		ASSERT_NO_THROW({solver->solve(dt, currentState, &newState);});

		EXPECT_TRUE(newState.getPositions().isApprox(yn_plus_1.position));
		EXPECT_TRUE(newState.getVelocities().isApprox(yn_plus_1.velocity));

		// 2nd time step
		integrateRK4(dt, m, yn_plus_1, &yn_plus_2);
		ASSERT_NO_THROW({solver->solve(dt, newState, &newState2);});

		EXPECT_TRUE(newState2.getPositions().isApprox(yn_plus_2.position));
		EXPECT_TRUE(newState2.getVelocities().isApprox(yn_plus_2.velocity));
	}

	// Test Runge Kutta 4 algorithm itself (with viscosity)
	// Test 2 iterations because Linear solvers have a different algorithm on the 1st pass from the following passes.
	{
		SCOPED_TRACE("RK4 with viscosityof 0.1");

		MassPoint m(0.1);
		MassPointState currentState, newState, newState2;
		currentState.getPositions().setLinSpaced(1.0, 3.0);
		currentState.getVelocities().setConstant(1.0);

		auto solver = std::make_shared<T>(&m);
		EXPECT_NO_THROW(m.setOdeSolver(solver));

		RungeKuttaState yn(currentState.getPositions(), currentState.getVelocities());
		RungeKuttaState yn_plus_1, yn_plus_2;

		EXPECT_TRUE(currentState.getPositions().isApprox(yn.position));
		EXPECT_TRUE(currentState.getVelocities().isApprox(yn.velocity));

		// 1st time step
		integrateRK4(dt, m, yn, &yn_plus_1);
		ASSERT_NO_THROW({solver->solve(dt, currentState, &newState);});
		EXPECT_TRUE(newState.getPositions().isApprox(yn_plus_1.position));
		EXPECT_TRUE(newState.getVelocities().isApprox(yn_plus_1.velocity));

		// 2nd time step
		integrateRK4(dt, m, yn_plus_1, &yn_plus_2);
		ASSERT_NO_THROW({solver->solve(dt, newState, &newState2);});
		EXPECT_TRUE(newState2.getPositions().isApprox(yn_plus_2.position));
		EXPECT_TRUE(newState2.getVelocities().isApprox(yn_plus_2.velocity));
	}
}
};

TEST(OdeSolverRungeKutta4, SolveTest)
{
	{
		SCOPED_TRACE("OdeSolverRungeKutta4 computing the compliance matrix");
		doSolveTest<OdeSolverRungeKutta4>();
	}
	{
		SCOPED_TRACE("OdeSolverLinearRungeKutta4 computing the compliance matrix");
		doSolveTest<OdeSolverLinearRungeKutta4>();
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

TEST(OdeSolverRungeKutta4, ComputeMatricesTest)
{
	{
		SCOPED_TRACE("RungeKutta4");
		doComputeMatricesTest<OdeSolverRungeKutta4>();
	}

	{
		SCOPED_TRACE("LinearRungeKutta4");
		doComputeMatricesTest<OdeSolverLinearRungeKutta4>();
	}
}

}; // Math

}; // SurgSim

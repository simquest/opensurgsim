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

namespace
{
template<class T>
void doConstructorTest()
{
	MassPoint m;
	ASSERT_NO_THROW({T solver(&m);});
}
};

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
		EXPECT_NO_THROW(m.setOdeSolver(solver));
		solver->setNewtonRaphsonMaximumIteration(1);

		// ma = mg <=> a = g
		// v(1) = g.dt + v(0)
		// x(1) = v(1).dt + x(0)
		ASSERT_NO_THROW({solver->solve(1e-3, state0, &state1);});
		EXPECT_EQ(defaultState, state0);
		EXPECT_NE(defaultState, state1);
		EXPECT_TRUE(state1.getVelocities().isApprox(m.m_gravity * 1e-3 + state0.getVelocities()));
		EXPECT_TRUE(state1.getPositions().isApprox(state1.getVelocities() * 1e-3 + state0.getPositions()));

		// v(2) = g.dt + v(1)
		// x(2) = v(2).dt + x(1)
		ASSERT_NO_THROW({solver->solve(1e-3, state1, &state2);});
		EXPECT_NE(defaultState, state1);
		EXPECT_NE(defaultState, state2);
		EXPECT_NE(state2, state1);
		EXPECT_TRUE(state2.getVelocities().isApprox(m.m_gravity * 1e-3 + state1.getVelocities()));
		EXPECT_TRUE(state2.getPositions().isApprox(state2.getVelocities() * 1e-3 + state1.getPositions()));
	}

	{
		MassPoint m(0.1);
		MassPointState defaultState, state0, state1, state2;
		auto solver = std::make_shared<T>(&m);
		EXPECT_NO_THROW(m.setOdeSolver(solver));
		solver->setNewtonRaphsonMaximumIteration(1);

		// ma = mg - c.v <=> a = g - c/m.v
		// v(1) = (g - c/m.v(1)).dt + v(0) <=> v(1) = I.(1.0 + dt.c/m)^-1.(g.dt + v(0))
		// x(1) = v(1).dt + x(0)
		ASSERT_NO_THROW({solver->solve(1e-3, state0, &state1);});
		EXPECT_EQ(defaultState, state0);
		EXPECT_NE(defaultState, state1);
		Matrix33d systemInverse = Matrix33d::Identity() * 1.0 / (1.0 + 1e-3 * 0.1 / m.m_mass);
		EXPECT_TRUE(state1.getVelocities().isApprox(systemInverse * (m.m_gravity * 1e-3 + state0.getVelocities())));
		EXPECT_TRUE(state1.getPositions().isApprox(state1.getVelocities() * 1e-3  + state0.getPositions()));

		// v(2) = (g - c/m.v(2)).dt + v(1) <=> v(2) = I.(1.0 + dt.c/m)^-1.(g.dt + v(1))
		// x(2) = v(2).dt + x(1)
		ASSERT_NO_THROW({solver->solve(1e-3, state1, &state2);});
		EXPECT_NE(defaultState, state1);
		EXPECT_NE(defaultState, state2);
		EXPECT_NE(state1, state2);
		EXPECT_TRUE(state2.getVelocities().isApprox(systemInverse * (m.m_gravity * 1e-3 + state1.getVelocities())));
		EXPECT_TRUE(state2.getPositions().isApprox(state2.getVelocities() * 1e-3 + state1.getPositions()));
	}
}
};

TEST(OdeSolverEulerImplicit, SolveTest)
{
	{
		SCOPED_TRACE("EulerImplicit computing the compliance matrix");
		doSolveTest<OdeSolverEulerImplicit>();
	}
	{
		SCOPED_TRACE("LinearEulerImplicit computing the compliance matrix");
		doSolveTest<OdeSolverLinearEulerImplicit>();
	}
}

namespace
{
template<class T>
void doComplexNonLinearOdeTest(size_t numNewtonRaphsonIteration, bool expectExactSolution)
{
	OdeComplexNonLinear odeEquation;
	MassPointState state0, state1, state2;
	state0.getPositions().setLinSpaced(1.4, 5.67);
	state0.getVelocities().setLinSpaced(-0.4, -0.3);
	double dt = 1e-3;
	auto solver = std::make_shared<T>(&odeEquation);
	EXPECT_NO_THROW(odeEquation.setOdeSolver(solver));
	solver->setNewtonRaphsonMaximumIteration(numNewtonRaphsonIteration);
	solver->setNewtonRaphsonEpsilonConvergence(1e-13);

	ASSERT_NO_THROW({solver->solve(dt, state0, &state1);});
	// We do 2 iterations as Linear ode solver will do the 1st iteration as a normal non-linear ode solver
	ASSERT_NO_THROW({solver->solve(dt, state1, &state2);});

	EXPECT_NE(state0, state1);
	EXPECT_NE(state1, state2);

	// The new state should verify the following equation:
	// {x(t+dt) = x(t) + dt.v(t+dt)
	// {v(t+dt) = v(t) + dt.M^-1.f(x(t+dt), v(t+dt))
	// with the notes that M = Id and f(x,v)=x.v^2
	// BUT the function f is non-linear enough that a single iteration of the Newton-Raphson won't produce
	// a solution close enough to the real solution of the problem. More iterations would be required to
	// refine the solution.
	auto xt = state1.getPositions();
	auto vt = state1.getVelocities();
	auto xt_plus_dt = state2.getPositions();
	auto vt_plus_dt = state2.getVelocities();
	auto ft_plus_dt = odeEquation.computeF(state2);
	Vector expectedVelocity = vt + dt * ft_plus_dt;

	// This is always true by construction
	EXPECT_TRUE(xt_plus_dt.isApprox(xt + dt * vt_plus_dt));

	if (!expectExactSolution)
	{
		EXPECT_FALSE(xt_plus_dt.isApprox(xt + dt * expectedVelocity));
		EXPECT_FALSE(vt_plus_dt.isApprox(expectedVelocity));
	}
	else
	{
		EXPECT_TRUE(xt_plus_dt.isApprox(xt + dt * expectedVelocity));
		EXPECT_TRUE(vt_plus_dt.isApprox(expectedVelocity));
	}
}
};

TEST(OdeSolverEulerImplicit, VerifyComplexNonLinearOdeTest)
{
	// OdeSolverEulerImplicit should succeed with enough number of Newton-Raphson iterations.
	{
		SCOPED_TRACE("A single Newton-Raphson iteration, using OdeSolverEulerImplicit");

		doComplexNonLinearOdeTest<OdeSolverEulerImplicit>(1, false);
	}

	{
		SCOPED_TRACE("Multiple Newton-Raphson iterations, using OdeSolverEulerImplicit");

		doComplexNonLinearOdeTest<OdeSolverEulerImplicit>(10, true);
	}

	// OdeSolverLinearEulerImplicit should fail no matter the number of Newton-Raphson iterations
	// it just does not handle non-linear problems.
	{
		SCOPED_TRACE("A single Newton-Raphson iteration, using OdeSolverLinearEulerImplicit");

		doComplexNonLinearOdeTest<OdeSolverLinearEulerImplicit>(1, false);
	}

	{
		SCOPED_TRACE("Multiple Newton-Raphson iterations, using OdeSolverLinearEulerImplicit");

		doComplexNonLinearOdeTest<OdeSolverLinearEulerImplicit>(10, false);
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

	Matrix expectedSystemMatrix = m.computeM(state) / dt + m.computeD(state) + dt * m.computeK(state);
	EXPECT_NO_THROW(solver->computeMatrices(dt, state));
	EXPECT_TRUE(solver->getSystemMatrix().isApprox(expectedSystemMatrix));
	EXPECT_TRUE(m.applyCompliance(state, Matrix::Identity(state.getNumDof(),
								  state.getNumDof())).isApprox(expectedSystemMatrix.inverse()));
}
};

TEST(OdeSolverEulerImplicit, ComputeMatricesTest)
{
	{
		SCOPED_TRACE("EulerImplicit");
		doComputeMatricesTest<OdeSolverEulerImplicit>();
	}

	{
		SCOPED_TRACE("LinearEulerImplicit");
		doComputeMatricesTest<OdeSolverLinearEulerImplicit>();
	}
}

}; // Math

}; // SurgSim

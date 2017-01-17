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

#include "Modules/Parallel/OdeSolverEulerImplicitOpenCl.h"
#include "SurgSim/Math/UnitTests/MockObject.h"

using SurgSim::Math::Matrix33d;

namespace SurgSim
{

namespace Parallel
{

namespace
{
template<class T>
void doConstructorTest()
{
	Math::MassPoint m;
	ASSERT_NO_THROW({T solver(&m);});
}
};

TEST(OdeSolverEulerImplicit, ConstructorTest)
{
	{
		SCOPED_TRACE("EulerImplicit");
		doConstructorTest<OdeSolverEulerImplicitOpenCl>();
	}
}

namespace
{
template<class T>
void doSolveTest(bool computeCompliance)
{
	// Test 2 iterations because Linear solvers have a different algorithm on the 1st pass from the following passes.

	{
		Math::MassPoint m;
		Math::MassPointState defaultState, state0, state1, state2;
		auto solver = std::make_shared<T>(&m);
		m.setOdeSolver(solver);
		solver->setNewtonRaphsonMaximumIteration(1);

		// ma = mg <=> a = g
		// v(1) = g.dt + v(0)
		// x(1) = v(1).dt + x(0)

		//IT LOOKS LIKE AT THE COPY OPERATION OF THE RHS to VCL_RHS that the VCL_RHS is not initialised or the problem
		//is that ViennalCL is not initialized

		ASSERT_NO_THROW({solver->solve(1e-3, state0, &state1, computeCompliance);});
		EXPECT_EQ(defaultState, state0);
		EXPECT_NE(defaultState, state1);
		EXPECT_TRUE(state1.getVelocities().isApprox(m.m_gravity * 1e-3 + state0.getVelocities()));
		EXPECT_TRUE(state1.getPositions().isApprox(state1.getVelocities() * 1e-3 + state0.getPositions()));

		// v(2) = g.dt + v(1)
		// x(2) = v(2).dt + x(1)
		ASSERT_NO_THROW({solver->solve(1e-3, state1, &state2, computeCompliance);});
		EXPECT_NE(defaultState, state1);
		EXPECT_NE(defaultState, state2);
		EXPECT_NE(state2, state1);
		EXPECT_TRUE(state2.getVelocities().isApprox(m.m_gravity * 1e-3 + state1.getVelocities()));
		EXPECT_TRUE(state2.getPositions().isApprox(state2.getVelocities() * 1e-3 + state1.getPositions()));
	}

	{
		Math::MassPoint m(0.1);
		Math::MassPointState defaultState, state0, state1, state2;
		auto solver = std::make_shared<T>(&m);
		m.setOdeSolver(solver);
		solver->setNewtonRaphsonMaximumIteration(1);

		// ma = mg - c.v <=> a = g - c/m.v
		// v(1) = (g - c/m.v(1)).dt + v(0) <=> v(1) = I.(1.0 + dt.c/m)^-1.(g.dt + v(0))
		// x(1) = v(1).dt + x(0)
		ASSERT_NO_THROW({solver->solve(1e-3, state0, &state1, computeCompliance);});
		EXPECT_EQ(defaultState, state0);
		EXPECT_NE(defaultState, state1);
		Matrix33d systemInverse = Matrix33d::Identity() * 1.0 / (1.0 + 1e-3 * 0.1 / m.m_mass);
		EXPECT_TRUE(state1.getVelocities().isApprox(systemInverse * (m.m_gravity * 1e-3 + state0.getVelocities())));
		EXPECT_TRUE(state1.getPositions().isApprox(state1.getVelocities() * 1e-3  + state0.getPositions()));

		// v(2) = (g - c/m.v(2)).dt + v(1) <=> v(2) = I.(1.0 + dt.c/m)^-1.(g.dt + v(1))
		// x(2) = v(2).dt + x(1)
		ASSERT_NO_THROW({solver->solve(1e-3, state1, &state2, computeCompliance);});
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
		doSolveTest<OdeSolverEulerImplicitOpenCl>(true);
	}
	{
		SCOPED_TRACE("EulerImplicit not computing the compliance matrix");
		doSolveTest<OdeSolverEulerImplicitOpenCl>(false);
	}
}

namespace
{
template<class T>
void doComplexNonLinearOdeTest(size_t numNewtonRaphsonIteration, bool expectExactSolution)
{
	Math::OdeComplexNonLinear odeEquation;
	Math::MassPointState state0, state1, state2;
	state0.getPositions().setLinSpaced(1.4, 5.67);
	state0.getVelocities().setLinSpaced(-0.4, -0.3);
	double dt = 1e-3;
	auto solver = std::make_shared<T>(&odeEquation);
	odeEquation.setOdeSolver(solver);
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
	odeEquation.updateFMDK(state2, Math::ODEEQUATIONUPDATE_F);
	auto ft_plus_dt = odeEquation.getF();
	Math::Vector expectedVelocity = vt + dt * ft_plus_dt;

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

		doComplexNonLinearOdeTest<OdeSolverEulerImplicitOpenCl>(1, false);
	}

	// OdeSolverLinearEulerImplicit should fail no matter the number of Newton-Raphson iterations
	// it just does not handle non-linear problems.
	{
		SCOPED_TRACE("A single Newton-Raphson iteration, using OdeSolverLinearEulerImplicit");

		doComplexNonLinearOdeTest<OdeSolverEulerImplicitOpenCl>(1, false);
	}

}

namespace
{
template <class T>
void doComputeMatricesTest()
{
	Math::MassPoint m;
	auto solver = std::make_shared<T>(&m);
	m.setOdeSolver(solver);
	Math::MassPointState state;
	double dt = 1e-3;

	m.updateFMDK(state, Math::ODEEQUATIONUPDATE_M | Math::ODEEQUATIONUPDATE_D | Math::ODEEQUATIONUPDATE_K);
	Math::Matrix expectedSystemMatrix = m.getM() / dt + m.getD() + dt * m.getK();
	EXPECT_NO_THROW(solver->computeMatrices(dt, state));
	EXPECT_TRUE(solver->getSystemMatrix().isApprox(expectedSystemMatrix));
	EXPECT_TRUE(solver->getComplianceMatrix().isApprox(expectedSystemMatrix.inverse()));
}
};

TEST(OdeSolverEulerImplicit, ComputeMatricesTest)
{
	{
		SCOPED_TRACE("EulerImplicit");
		doComputeMatricesTest<OdeSolverEulerImplicitOpenCl>();
	}
}

}; // Math

}; // SurgSim

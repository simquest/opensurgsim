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

#include "SurgSim/Math/OdeEquation.h"
#include "SurgSim/Math/UnitTests/MockObject.h"

using SurgSim::Math::MassPoint;
using SurgSim::Math::MassPointState;

TEST(OdeEquationTests, Constructor)
{
	ASSERT_NO_THROW({MassPoint m;});
	ASSERT_NO_THROW({MassPoint* m = new MassPoint; delete m;});
	ASSERT_NO_THROW({MassPoint* m = new MassPoint[10]; delete [] m;});
	ASSERT_NO_THROW({std::shared_ptr<MassPoint> m = std::make_shared<MassPoint>(); });
}

TEST(OdeEquationTests, GetInitialStateTest)
{
	MassPoint m;
	ASSERT_NE(nullptr, m.getInitialState());
	EXPECT_EQ(3, m.getInitialState()->getPositions().size());
	EXPECT_EQ(3, m.getInitialState()->getVelocities().size());
	SurgSim::Math::Vector3d expectedX = SurgSim::Math::Vector3d::LinSpaced(1.0, 1.3);
	EXPECT_TRUE(m.getInitialState()->getPositions().isApprox(expectedX));
	SurgSim::Math::Vector3d expectedV = SurgSim::Math::Vector3d::LinSpaced(0.4, -0.3);
	EXPECT_TRUE(m.getInitialState()->getVelocities().isApprox(expectedV));
}

TEST(OdeEquationTests, ComputesTest)
{
	{
		SCOPED_TRACE("OdeEquationTests computes tests with 0 viscosity");
		MassPoint m;
		MassPointState state;
		state.getPositions() = SurgSim::Math::Vector3d(1.0, 1.0, 1.0);
		state.getVelocities() = SurgSim::Math::Vector3d(1.0, 1.0, 1.0);

		SurgSim::Math::Vector3d expectedF = m.m_gravity * m.m_mass;
		SurgSim::Math::Matrix33d expectedM = m.m_mass * SurgSim::Math::Matrix33d::Identity();
		SurgSim::Math::Matrix33d expectedD = SurgSim::Math::Matrix33d::Zero();
		SurgSim::Math::Matrix33d expectedK = SurgSim::Math::Matrix33d::Zero();
		EXPECT_TRUE(m.computeF(state).isApprox(expectedF));
		EXPECT_TRUE(m.computeM(state).isApprox(expectedM));
		EXPECT_TRUE(m.computeD(state).isApprox(expectedD));
		EXPECT_TRUE(m.computeK(state).isApprox(expectedK));
		{
			SurgSim::Math::Vector* F = nullptr;
			SurgSim::Math::SparseMatrix* M = nullptr;
			SurgSim::Math::SparseMatrix* D = nullptr;
			SurgSim::Math::SparseMatrix* K = nullptr;

			m.computeFMDK(state, &F, &M, &D, &K);
			ASSERT_NE(nullptr, F);
			EXPECT_TRUE(F->isApprox(expectedF));
			ASSERT_NE(nullptr, M);
			EXPECT_TRUE(M->isApprox(expectedM));
			ASSERT_NE(nullptr, D);
			EXPECT_TRUE(D->isApprox(expectedD));
			ASSERT_NE(nullptr, K);
			EXPECT_TRUE(K->isApprox(expectedK));
		}
	}

	{
		SCOPED_TRACE("OdeEquationTests computes tests with non 0 viscosity");
		MassPoint m(0.1);
		MassPointState state;
		state.getPositions() = SurgSim::Math::Vector3d(1.0, 1.0, 1.0);
		state.getVelocities() = SurgSim::Math::Vector3d(1.0, 1.0, 1.0);

		SurgSim::Math::Vector3d expectedF = m.m_gravity * m.m_mass - 0.1 * state.getVelocities();
		SurgSim::Math::Matrix33d expectedM = m.m_mass * SurgSim::Math::Matrix33d::Identity();
		SurgSim::Math::Matrix33d expectedD = 0.1 * SurgSim::Math::Matrix33d::Identity();
		SurgSim::Math::Matrix33d expectedK = SurgSim::Math::Matrix33d::Zero();
		EXPECT_TRUE(m.computeF(state).isApprox(expectedF));
		EXPECT_TRUE(m.computeM(state).isApprox(expectedM));
		EXPECT_TRUE(m.computeD(state).isApprox(expectedD));
		EXPECT_TRUE(m.computeK(state).isApprox(expectedK));
		{
			SurgSim::Math::Vector* F = nullptr;
			SurgSim::Math::SparseMatrix* M = nullptr;
			SurgSim::Math::SparseMatrix* D = nullptr;
			SurgSim::Math::SparseMatrix* K = nullptr;

			m.computeFMDK(state, &F, &M, &D, &K);
			ASSERT_NE(nullptr, F);
			EXPECT_TRUE(F->isApprox(expectedF));
			ASSERT_NE(nullptr, M);
			EXPECT_TRUE(M->isApprox(expectedM));
			ASSERT_NE(nullptr, D);
			EXPECT_TRUE(D->isApprox(expectedD));
			ASSERT_NE(nullptr, K);
			EXPECT_TRUE(K->isApprox(expectedK));
		}
	}
}

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
		m.updateFMDK(state, SurgSim::Math::ODEEQUATIONUPDATE_F);
		EXPECT_TRUE(m.getF().isApprox(expectedF));
		m.updateFMDK(state, SurgSim::Math::ODEEQUATIONUPDATE_M);
		EXPECT_TRUE(m.getM().isApprox(expectedM));
		m.updateFMDK(state, SurgSim::Math::ODEEQUATIONUPDATE_D);
		EXPECT_TRUE(m.getD().isApprox(expectedD));
		m.updateFMDK(state, SurgSim::Math::ODEEQUATIONUPDATE_K);
		EXPECT_TRUE(m.getK().isApprox(expectedK));
		{
			m.updateFMDK(state, SurgSim::Math::ODEEQUATIONUPDATE_FMDK);
			EXPECT_TRUE(m.getF().isApprox(expectedF));
			EXPECT_TRUE(m.getM().isApprox(expectedM));
			EXPECT_TRUE(m.getD().isApprox(expectedD));
			EXPECT_TRUE(m.getK().isApprox(expectedK));
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
		m.updateFMDK(state, SurgSim::Math::ODEEQUATIONUPDATE_F);
		EXPECT_TRUE(m.getF().isApprox(expectedF));
		m.updateFMDK(state, SurgSim::Math::ODEEQUATIONUPDATE_M);
		EXPECT_TRUE(m.getM().isApprox(expectedM));
		m.updateFMDK(state, SurgSim::Math::ODEEQUATIONUPDATE_D);
		EXPECT_TRUE(m.getD().isApprox(expectedD));
		m.updateFMDK(state, SurgSim::Math::ODEEQUATIONUPDATE_K);
		EXPECT_TRUE(m.getK().isApprox(expectedK));
		{
			m.updateFMDK(state, SurgSim::Math::ODEEQUATIONUPDATE_FMDK);
			EXPECT_TRUE(m.getF().isApprox(expectedF));
			EXPECT_TRUE(m.getM().isApprox(expectedM));
			EXPECT_TRUE(m.getD().isApprox(expectedD));
			EXPECT_TRUE(m.getK().isApprox(expectedK));
		}
	}
}

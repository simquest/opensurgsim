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

#include <gtest/gtest.h>

#include <string>

#include <SurgSim/Physics/MassSpringRepresentation.h>

using SurgSim::Physics::MassSpringRepresentation;
using SurgSim::Math::Vector3d;

class MassSpringRepresentationTests : public ::testing::Test
{
public:
	void SetUp() override
	{
		// Initialization values for the simulation
		m_dt = 1e-3;

		// Initialization values for 1D case
		m_extremities1D[0] = Vector3d(-1.0, 0.0, 0.0);
		m_extremities1D[1] = Vector3d( 1.0, 0.0, 0.0);
		m_numNodesPerDim1D[0] = 10;
		m_totalMass1D = 0.01;
		m_springStiffness1D = 1.0;
		m_springDamping1D = 0.2;

		// Initialization values for 2D case
		m_extremities2D[0][0] = Vector3d(-1.0, -1.0, 0.0);
		m_extremities2D[0][1] = Vector3d( 1.0, -1.0, 0.0);
		m_extremities2D[1][0] = Vector3d(-1.0,  1.0, 0.0);
		m_extremities2D[1][1] = Vector3d( 1.0,  1.0, 0.0);
		m_numNodesPerDim1D[0] = 5;
		m_numNodesPerDim1D[1] = 5;
		m_totalMass1D = 0.05;
		m_springStiffness1D = 1.0;
		m_springDamping1D = 0.2;

		// Initialization values for 3D case
		m_extremities3D[0][0][0] = Vector3d(-1.0, -1.0, -1.0);
		m_extremities3D[0][0][1] = Vector3d( 1.0, -1.0, -1.0);
		m_extremities3D[0][1][0] = Vector3d(-1.0,  1.0, -1.0);
		m_extremities3D[0][1][1] = Vector3d( 1.0,  1.0, -1.0);
		m_extremities3D[1][0][0] = Vector3d(-1.0, -1.0,  1.0);
		m_extremities3D[1][0][1] = Vector3d( 1.0, -1.0,  1.0);
		m_extremities3D[1][1][0] = Vector3d(-1.0,  1.0,  1.0);
		m_extremities3D[1][1][1] = Vector3d( 1.0,  1.0,  1.0);
		m_numNodesPerDim1D[0] = 4;
		m_numNodesPerDim1D[1] = 4;
		m_numNodesPerDim1D[2] = 4;
		m_totalMass1D = 0.1;
		m_springStiffness1D = 1.0;
		m_springDamping1D = 0.2;
	}

	void TearDown() override
	{

	}

protected:
	/// Simulation parameters
	double m_dt;

	/// Initialization 1D case (extremities)
	Vector3d m_extremities1D[2];
	/// Initialization 1D case (number of nodes per dimension)
	int m_numNodesPerDim1D[1];
	/// Initialization 1D case (total mass in Kg)
	double m_totalMass1D;
	/// Initialization 1D case (spring stiffness and damping)
	double m_springStiffness1D, m_springDamping1D;

	/// Initialization 2D case (extremities)
	Vector3d m_extremities2D[2][2];
	/// Initialization 2D case (number of nodes per dimension)
	int m_numNodesPerDim2D[2];
	/// Initialization 2D case (total mass in Kg)
	double m_totalMass2D;
	/// Initialization 2D case (spring stiffness and damping)
	double m_springStiffness2D, m_springDamping2D;

	/// Initialization 3D case (extremities)
	Vector3d m_extremities3D[2][2][2];
	/// Initialization 3D case (number of nodes per dimension)
	int m_numNodesPerDim3D[3];
	/// Initialization 3D case (total mass in Kg)
	double m_totalMass3D;
	/// Initialization 3D case (spring stiffness and damping)
	double m_springStiffness3D, m_springDamping3D;
};

TEST_F(MassSpringRepresentationTests, Constructor)
{
	ASSERT_NO_THROW({MassSpringRepresentation m("MassSpring");});

	ASSERT_NO_THROW({MassSpringRepresentation* m = new MassSpringRepresentation("MassSpring"); delete m;});
}

TEST_F(MassSpringRepresentationTests, Init1D)
{
	std::string name("MassSpring");
	MassSpringRepresentation m(name);

	m.init1D(m_extremities1D, m_numNodesPerDim1D, m_totalMass1D, m_springStiffness1D, m_springDamping1D);
	EXPECT_EQ(name, m.getName());
	EXPECT_EQ(m_numNodesPerDim1D[0] * 3, m.getNumDof());
	EXPECT_EQ(m_numNodesPerDim1D[0], m.getNumMasses());
	EXPECT_EQ(m_numNodesPerDim1D[0] - 1, m.getNumSprings());
	EXPECT_EQ(0u, m.getNumBoundaryConditions());
	EXPECT_EQ(MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER, m.getIntegrationScheme());
	EXPECT_DOUBLE_EQ(0.0, m.getRayleighDampingMass());
	EXPECT_DOUBLE_EQ(0.0, m.getRayleighDampingStiffness());
	EXPECT_DOUBLE_EQ(m_totalMass1D, m.getTotalMass());
	EXPECT_TRUE(m.getInitialPose().isApprox(SurgSim::Math::RigidTransform3d::Identity()));
	EXPECT_TRUE(m.getPose().isApprox(SurgSim::Math::RigidTransform3d::Identity()));

	Vector3d startPoint = m_extremities1D[0];
	Vector3d delta = (m_extremities1D[1] - m_extremities1D[0]) / static_cast<double>(m_numNodesPerDim1D[0] - 1);
	for (unsigned int nodeId = 0; nodeId < m.getNumMasses(); nodeId++)
	{
		EXPECT_DOUBLE_EQ(m_totalMass1D / m_numNodesPerDim1D[0], m.getMassParameter(nodeId).getMass());
		EXPECT_TRUE(m.getMassParameter(nodeId).getVelocity().isApprox(Vector3d::Zero()));
		EXPECT_TRUE(m.getFinalState().getVertex(nodeId).position.isApprox(startPoint + nodeId * delta));
	}

	for (unsigned int springId = 0; springId < m.getNumSprings(); springId++)
	{
		EXPECT_DOUBLE_EQ(m_springStiffness1D, m.getSpringParameter(springId).getStiffness());
		EXPECT_DOUBLE_EQ(m_springDamping1D, m.getSpringParameter(springId).getDamping());
		EXPECT_DOUBLE_EQ(delta.norm(), m.getSpringParameter(springId).getInitialLength());
	}
}

TEST_F(MassSpringRepresentationTests, Init2D)
{
	MassSpringRepresentation massSpring("MassSpring");

	massSpring.init2D(m_extremities2D, m_numNodesPerDim2D, m_totalMass2D, m_springStiffness2D, m_springDamping2D);
	//EXPECT_EQ(m_numNodesPerDim2D[0] * m_numNodesPerDim2D[1] * 3, massSpring.getNumDof());
}

TEST_F(MassSpringRepresentationTests, Init3D)
{
	MassSpringRepresentation massSpring("MassSpring");

	massSpring.init3D(m_extremities3D, m_numNodesPerDim3D, m_totalMass3D, m_springStiffness3D, m_springDamping3D);
	//EXPECT_EQ(m_numNodesPerDim3D[0] * m_numNodesPerDim3D[1] * m_numNodesPerDim3D[2] * 3, massSpring.getNumDof());
}

TEST_F(MassSpringRepresentationTests, NoGravityTest)
{
	MassSpringRepresentation m("MassSpring");

	m.init1D(m_extremities1D, m_numNodesPerDim1D, m_totalMass1D, m_springStiffness1D, m_springDamping1D);
	m.setIsActive(true);
	m.setIsGravityEnabled(false);
	m.setIntegrationScheme(MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER);

	m.beforeUpdate(m_dt);
	m.update(m_dt);
	m.afterUpdate(m_dt);
}

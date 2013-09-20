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
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Physics::MassSpringRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

class MockMassSpring : public MassSpringRepresentation
{
public:
	typedef MassSpringRepresentation::Vector Vector;

	MockMassSpring(const std::string& name): MassSpringRepresentation(name)
	{}

	virtual ~MockMassSpring()
	{}

	const Vector& getCurrentPositionEigenState() const { return m_x; }
	const Vector& getPreviousPositionEigenState() const { return m_xPrevious; }
	const Vector& getCurrentVelocityEigenState() const { return m_v; }

	const Vector3d& getGravityVector() const { return getGravity(); }
};

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
		m_springStiffness1D = 10.0;
		m_springDamping1D = 0.2;

		// Initialization values for 2D case
		m_extremities2D[0][0] = Vector3d(-1.0, -1.0, 0.0);
		m_extremities2D[0][1] = Vector3d( 1.0, -1.0, 0.0);
		m_extremities2D[1][0] = Vector3d(-1.0,  1.0, 0.0);
		m_extremities2D[1][1] = Vector3d( 1.0,  1.0, 0.0);
		m_numNodesPerDim2D[0] = 5;
		m_numNodesPerDim2D[1] = 5;
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
		m_numNodesPerDim3D[0] = 4;
		m_numNodesPerDim3D[1] = 4;
		m_numNodesPerDim3D[2] = 4;
		m_totalMass1D = 0.1;
		m_springStiffness1D = 1.0;
		m_springDamping1D = 0.2;

		// Poses
		m_poseIdentity.setIdentity();
		Quaterniond q;
		q.coeffs().setRandom();
		q.normalize();
		Vector3d t;
		t.setRandom();
		m_poseRandom = SurgSim::Math::makeRigidTransform(q, t);
	}

	void TearDown() override
	{

	}

	void FallingTest(MockMassSpring &m);

protected:
	/// Simulation parameters
	double m_dt;

	/// Initialization 1D case (extremities)
	Vector3d m_extremities1D[2];
	/// Initialization 1D case (number of nodes per dimension)
	unsigned int m_numNodesPerDim1D[1];
	/// Initialization 1D case (total mass in Kg)
	double m_totalMass1D;
	/// Initialization 1D case (spring stiffness and damping)
	double m_springStiffness1D, m_springDamping1D;

	/// Initialization 2D case (extremities)
	Vector3d m_extremities2D[2][2];
	/// Initialization 2D case (number of nodes per dimension)
	unsigned int m_numNodesPerDim2D[2];
	/// Initialization 2D case (total mass in Kg)
	double m_totalMass2D;
	/// Initialization 2D case (spring stiffness and damping)
	double m_springStiffness2D, m_springDamping2D;

	/// Initialization 3D case (extremities)
	Vector3d m_extremities3D[2][2][2];
	/// Initialization 3D case (number of nodes per dimension)
	unsigned int m_numNodesPerDim3D[3];
	/// Initialization 3D case (total mass in Kg)
	double m_totalMass3D;
	/// Initialization 3D case (spring stiffness and damping)
	double m_springStiffness3D, m_springDamping3D;

	/// IdentityPose and random pose
	SurgSim::Math::RigidTransform3d m_poseIdentity, m_poseRandom;
};

TEST_F(MassSpringRepresentationTests, Constructor)
{
	ASSERT_NO_THROW({MassSpringRepresentation m("MassSpring");});

	ASSERT_NO_THROW({MassSpringRepresentation* m = new MassSpringRepresentation("MassSpring"); delete m;});
}

TEST_F(MassSpringRepresentationTests, Init1D)
{
	std::string name("MassSpring");
	MockMassSpring m(name);

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
		EXPECT_TRUE(m.getInitialState().getVertex(nodeId).position.isApprox(startPoint + nodeId * delta));
		EXPECT_TRUE(m.getFinalState().getVertex(nodeId).position.isApprox(startPoint + nodeId * delta));
		EXPECT_TRUE(m.getCurrentPositionEigenState().segment(3 * nodeId, 3).isApprox(startPoint + nodeId * delta));
		EXPECT_TRUE(m.getPreviousPositionEigenState().segment(3 * nodeId, 3).isApprox(startPoint + nodeId * delta));
		EXPECT_TRUE(m.getCurrentVelocityEigenState().segment(3 * nodeId, 3).isApprox(Vector3d::Zero()));
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
	// To be completed
}

TEST_F(MassSpringRepresentationTests, Init3D)
{
	MassSpringRepresentation massSpring("MassSpring");

	massSpring.init3D(m_extremities3D, m_numNodesPerDim3D, m_totalMass3D, m_springStiffness3D, m_springDamping3D);
	// To be completed
}

TEST_F(MassSpringRepresentationTests, SetGetMethods)
{
	using SurgSim::Physics::MassParameter;
	using SurgSim::Physics::LinearSpringParameter;

	MockMassSpring m("MassSpring");

	m.init1D(m_extremities1D, m_numNodesPerDim1D, m_totalMass1D, m_springStiffness1D, m_springDamping1D);

	m.setInitialPose(m_poseRandom);
	EXPECT_TRUE(m.getInitialPose().isApprox(m_poseRandom));

	m.setIntegrationScheme(MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER);
	EXPECT_EQ(MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER, m.getIntegrationScheme());
	m.setIntegrationScheme(MassSpringRepresentation::INTEGRATIONSCHEME_IMPLICIT_EULER);
	EXPECT_EQ(MassSpringRepresentation::INTEGRATIONSCHEME_IMPLICIT_EULER, m.getIntegrationScheme());

	EXPECT_TRUE(m.getPose().isApprox(m_poseIdentity));
	EXPECT_THROW(m.setPose(m_poseIdentity), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(m.setPose(m_poseRandom), SurgSim::Framework::AssertionFailure);
	EXPECT_TRUE(m.getPose().isApprox(m_poseIdentity));

	m.setRayleighDampingMass(5.5);
	EXPECT_DOUBLE_EQ(5.5, m.getRayleighDampingMass());
	m.setRayleighDampingStiffness(5.4);
	EXPECT_DOUBLE_EQ(5.4, m.getRayleighDampingStiffness());

	EXPECT_EQ(0u, m.getNumBoundaryConditions());
	EXPECT_NO_THROW(m.addBoundaryCondition(0));
	EXPECT_EQ(1u, m.getNumBoundaryConditions());
	EXPECT_NO_THROW(m.addBoundaryCondition(5));
	EXPECT_EQ(2u, m.getNumBoundaryConditions());

	unsigned int val;
	EXPECT_NO_THROW(val = m.getBoundaryCondition(0));
	EXPECT_EQ(0u, val);
	EXPECT_NO_THROW(val = m.getBoundaryCondition(1));
	EXPECT_EQ(5u, val);
	EXPECT_THROW(val = m.getBoundaryCondition(2), SurgSim::Framework::AssertionFailure);

	// Check that mass can be set/get
	MassParameter& massParam = m.getMassParameter(0);
	EXPECT_DOUBLE_EQ(m_totalMass1D / static_cast<double>(m_numNodesPerDim1D[0]), massParam.getMass());
	massParam.setMass(10.0);
	EXPECT_DOUBLE_EQ(10.0, massParam.getMass());
	EXPECT_TRUE(massParam.getVelocity().isApprox(Vector3d::Zero()));
	massParam.setVelocity(Vector3d::UnitY());
	EXPECT_TRUE(massParam.getVelocity().isApprox(Vector3d::UnitY()));

	// check that spring parameters can be set/get
	Vector3d delta = (m_extremities1D[1] - m_extremities1D[0]) / static_cast<double>(m_numNodesPerDim1D[0] - 1);
	LinearSpringParameter& springParam = m.getSpringParameter(0);
	EXPECT_DOUBLE_EQ(delta.norm(), springParam.getInitialLength());
	springParam.setInitialLength(10.0);
	EXPECT_DOUBLE_EQ(10.0, springParam.getInitialLength());
	EXPECT_DOUBLE_EQ(m_springStiffness1D, springParam.getStiffness());
	springParam.setStiffness(11.0);
	EXPECT_DOUBLE_EQ(11.0, springParam.getStiffness());
	EXPECT_DOUBLE_EQ(m_springDamping1D, springParam.getDamping());
	springParam.setDamping(12.0);
	EXPECT_DOUBLE_EQ(12.0, springParam.getDamping());
}

TEST_F(MassSpringRepresentationTests, Model1DNoGravityTest)
{
	MockMassSpring m("MassSpring 1D");

	m.init1D(m_extremities1D, m_numNodesPerDim1D, m_totalMass1D, m_springStiffness1D, m_springDamping1D);
	m.setIsActive(true);
	m.setIsGravityEnabled(false);
	m.setIntegrationScheme(MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER);

	m.beforeUpdate(m_dt);
	m.update(m_dt);
	m.afterUpdate(m_dt);

	Vector3d startPoint = m_extremities1D[0];
	Vector3d delta = (m_extremities1D[1] - m_extremities1D[0]) / static_cast<double>(m_numNodesPerDim1D[0] - 1);
	for (unsigned int nodeId = 0; nodeId < m.getNumMasses(); nodeId++)
	{
		EXPECT_TRUE(m.getFinalState().getVertex(nodeId).position.isApprox(startPoint + nodeId * delta));
		EXPECT_TRUE(m.getCurrentPositionEigenState().segment(3 * nodeId, 3).isApprox(startPoint + nodeId * delta));
		EXPECT_TRUE(m.getPreviousPositionEigenState().segment(3 * nodeId, 3).isApprox(startPoint + nodeId * delta));
		EXPECT_TRUE(m.getCurrentVelocityEigenState().segment(3 * nodeId, 3).isZero());
	}
}

TEST_F(MassSpringRepresentationTests, Model2DNoGravityTest)
{
	// To be completed
}

TEST_F(MassSpringRepresentationTests, Model3DNoGravityTest)
{
	// To be completed
}


TEST_F(MassSpringRepresentationTests, OneSpringFrequencyTest)
{
	MockMassSpring m("MassSpring");

	// Simulate 1 mass connected to 1 spring (no gravity, no damping)
	m_numNodesPerDim1D[0] = 2;
	m_springDamping1D = 0.0;
	m.init1D(m_extremities1D, m_numNodesPerDim1D, m_totalMass1D, m_springStiffness1D, m_springDamping1D);
	// Fix the 1st node (Boundary Condition), only 1 mass is free
	m.addBoundaryCondition(0);
	// Pull on the free mass, by simply making the initial length shorter (creating an extension right away)
	m.getSpringParameter(0).setInitialLength(0.0);
	m.setIsActive(true);
	m.setIsGravityEnabled(false);
	// Only the Modified Euler Explicit integration conserves the energy exactly
	// (explicit adds energy to the system, implicit removes energy to the system)
	m.setIntegrationScheme(MassSpringRepresentation::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER);

	// Frequency = 1/(2PI) sqrt(k / m)
	double f = 1.0/(2.0 * M_PI) * sqrt(m_springStiffness1D / (m_totalMass1D/2.0));
	// Period = 1/f
	double period = 1.0 / f;
	// Let's look for a time step, factor of period, small enough for stability of Explicit Euler (~1Khz)
	m_dt = period;
	while(m_dt > 1e-3) m_dt *= 0.5;

	// Simulate a single mass connected to a spring at the same time for comparison purpose
	Vector3d x = m_extremities1D[1];
	Vector3d v = Vector3d::Zero();

	double time = 0.0;
	// Let's do all iterations (except the last 2) testing that the mass is NOT back yet to its original position
	while(time < period - 2.0*m_dt)
	{
		m.beforeUpdate(m_dt);
		m.update(m_dt);
		m.afterUpdate(m_dt);

		// Manually simulate a single mass connected to a spring
		Vector3d f = ( m_springStiffness1D * (m_extremities1D[0] - x) ) / (m_totalMass1D/2.0);
		v += f * m_dt;
		x += v * m_dt;
		
		const Vector3d& finalMeshPosition = m.getFinalState().getVertex(1).position;
		EXPECT_TRUE(finalMeshPosition.isApprox(m.getCurrentPositionEigenState().segment(3 * 1, 3)));
		EXPECT_TRUE(finalMeshPosition.isApprox(x, 1e-8));
		EXPECT_TRUE(m.getFinalState().getVertex(1).data.getVelocity().isApprox(v, 1e-8));

		Vector3d deltaCompare = m.getFinalState().getVertex(1).position - m_extremities1D[1];
		EXPECT_FALSE(deltaCompare.isZero(1e-9)) << "Error is " << deltaCompare.norm();
		deltaCompare = m.getCurrentPositionEigenState().segment(3 * 1, 3) - m_extremities1D[1];
		EXPECT_FALSE(deltaCompare.isZero(1e-9)) << "Error is " << deltaCompare.norm();
		
		time += m_dt;
	}

	// Let's do the last 2 iterations without testing as we are getting close enough to the solution
	// that the test might become true before reaching the very last iteration
	while(time < period)
	{
		m.beforeUpdate(m_dt);
		m.update(m_dt);
		m.afterUpdate(m_dt);
		time += m_dt;
	}

	Vector3d deltaCompare = m.getFinalState().getVertex(1).position - m_extremities1D[1];
	EXPECT_TRUE(deltaCompare.isZero(8e-9)) << "Error is " << deltaCompare.norm();
	deltaCompare = m.getCurrentPositionEigenState().segment(3 * 1, 3) - m_extremities1D[1];
	EXPECT_TRUE(deltaCompare.isZero(8e-9)) << "Error is " << deltaCompare.norm();
}

void MassSpringRepresentationTests::FallingTest(MockMassSpring &m)
{
	m.setIsActive(true);
	m.setIsGravityEnabled(true);
	m.setIntegrationScheme(MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER);

	// run few iterations of simulation...
	for (int i = 0; i< 5; i++)
	{
		m.beforeUpdate(m_dt);
		m.update(m_dt);
		m.afterUpdate(m_dt);

		// Making sure that each mass has a velocity directed toward the gravity vector direction
		// with no orthogonal components
		for (unsigned int nodeId = 0; nodeId < m.getNumMasses(); nodeId++)
		{
			EXPECT_TRUE(m.getFinalState().getVertex(nodeId).data.getVelocity().dot(m.getGravityVector()) > 0.0);
			EXPECT_TRUE(m.getFinalState().getVertex(nodeId).data.getVelocity().cross(m.getGravityVector()).isZero());
		}
	}
}

TEST_F(MassSpringRepresentationTests, Model1DFallingTest)
{
	MockMassSpring m("MassSpring 1D");
	m.init1D(m_extremities1D, m_numNodesPerDim1D, m_totalMass1D, m_springStiffness1D, m_springDamping1D);
	FallingTest(m);
}

TEST_F(MassSpringRepresentationTests, Model2DFallingTest)
{
	MockMassSpring m("MassSpring 2D");
	m.init2D(m_extremities2D, m_numNodesPerDim2D, m_totalMass2D, m_springStiffness2D, m_springDamping2D);
	FallingTest(m);
}

TEST_F(MassSpringRepresentationTests, Model3DFallingTest)
{
	MockMassSpring m("MassSpring 3D");
	m.init3D(m_extremities3D, m_numNodesPerDim3D, m_totalMass3D, m_springStiffness3D, m_springDamping3D);
	FallingTest(m);
}

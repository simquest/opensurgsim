// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::LinearSpring;
using SurgSim::Physics::MassSpringRepresentation;
using SurgSim::Physics::MockMassSpring;

namespace
{
	const double epsilon = 1e-10;
};

class MassSpringMechanicalValidationTests : public ::testing::Test
{
public:
	void SetUp() override
	{
		// Initialization values for the simulation
		m_dt = 1e-3;

		m_numNodes = 10;
		m_totalMass = 0.1;
		m_springStiffness = 1.0;
		m_springDamping = 0.1;
		m_rayleighDampingMass = 1e0;
		m_rayleighDampingStiffness = 1e-3;
		m_nodeBoundaryConditions.push_back(0);

		// Poses
		m_poseIdentity.setIdentity();
		Quaterniond q;
		q.coeffs().setRandom();
		q.normalize();
		Vector3d t;
		t.setRandom();
		m_poseRandom = SurgSim::Math::makeRigidTransform(q, t);
	}

	/// Run the energy test on a spring
	/// \param scheme The numerical integration scheme to be used
	/// \param expectedBehavior -1 if the energy should decrease, 1 if it should increase, 0 if stable
	void runEnergyTest(SurgSim::Math::IntegrationScheme scheme, int expectedBehavior)
	{
		m_dt = 1e-3;

		// 2 nodes, the 1st node fixed, no Rayleigh damping and no spring damping
		// A spring oscillating, only subject to mass and stiffness
		m_numNodes = 2;
		m_totalMass = 0.1;
		m_springStiffness = 1.0;
		m_springDamping = 0.0;
		m_rayleighDampingMass = 0.0;
		m_rayleighDampingStiffness = 0.0;
		m_nodeBoundaryConditions.clear();
		m_nodeBoundaryConditions.push_back(0);

		MockMassSpring m("MassSpring", m_poseRandom, m_numNodes, m_nodeBoundaryConditions, m_totalMass,
			m_rayleighDampingMass, m_rayleighDampingStiffness, m_springStiffness, m_springDamping,
			scheme);

		m.initialize(std::make_shared<SurgSim::Framework::Runtime>());
		m.wakeUp();

		// Pull on the free mass, by simply making the initial length shorter (creating an extension right away)
		std::static_pointer_cast<LinearSpring>(m.getSpring(0))->setRestLength(0.0);
		m.setIsGravityEnabled(false);

		Vector3d initialDelta = m.getCurrentState()->getPosition(1) - m.getCurrentState()->getPosition(0);
		double initialEk = 0.5 * (m_totalMass/2.0) * m.getCurrentState()->getVelocity(1).squaredNorm();
		double initialEs = 0.5 * m_springStiffness * initialDelta.squaredNorm();
		double initialEnergy = initialEk + initialEs;

		// Simulate 2 seconds of virtual time
		double time = 0.0;
		while(time < 2.0)
		{
			m.beforeUpdate(m_dt);
			m.update(m_dt);
			m.afterUpdate(m_dt);

			Vector3d previousDelta = m.getPreviousState()->getPosition(1) - m.getPreviousState()->getPosition(0);
			Vector3d currentDelta = m.getCurrentState()->getPosition(1) - m.getCurrentState()->getPosition(0);

			// Calculate previous/current system energy (kinetic energy + spring energy)
			double previousEk = 0.5 * (m_totalMass/2.0) * m.getPreviousState()->getVelocity(1).squaredNorm();
			double previousEs = 0.5 * m_springStiffness * previousDelta.squaredNorm();
			double previousEnergy = previousEk + previousEs;
			double currentEk = 0.5 * (m_totalMass/2.0) * m.getCurrentState()->getVelocity(1).squaredNorm();
			double currentEs = 0.5 * m_springStiffness * currentDelta.squaredNorm();
			double currentEnergy = currentEk + currentEs;
			if (expectedBehavior < 0)
			{
				EXPECT_LT(currentEnergy, previousEnergy);
			}
			else if (expectedBehavior > 0)
			{
				EXPECT_GT(currentEnergy, previousEnergy);
			}
			else
			{
				EXPECT_NEAR(currentEnergy, previousEnergy, 2.6e-6);
				EXPECT_NEAR(currentEnergy, initialEnergy, 3e-4);
			}

			time += m_dt;
		}
	}

protected:
	/// Simulation parameters
	double m_dt;

	/// Number of nodes
	size_t m_numNodes;
	/// NodeIds boundary conditions
	std::vector<size_t> m_nodeBoundaryConditions;
	/// Total mass (in Kg)
	double m_totalMass;
	/// Spring stiffness and damping
	double m_springStiffness, m_springDamping;
	/// Rayleigh damping (mass and stiffness coefs)
	double m_rayleighDampingMass, m_rayleighDampingStiffness;

	/// IdentityPose and random pose
	SurgSim::Math::RigidTransform3d m_poseIdentity, m_poseRandom;
};

TEST_F(MassSpringMechanicalValidationTests, NoGravityTest)
{
	// Note the use of identity pose to avoid small variation between the spring rest length and current length
	MockMassSpring m("MassSpring", m_poseIdentity, m_numNodes, m_nodeBoundaryConditions, m_totalMass,
		m_rayleighDampingMass, m_rayleighDampingStiffness, m_springStiffness, m_springDamping,
		SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT);

	m.setIsGravityEnabled(false);

	m.initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m.wakeUp();

	m.beforeUpdate(m_dt);
	m.update(m_dt);
	m.afterUpdate(m_dt);

	EXPECT_EQ(*m.getInitialState(), *m.getFinalState());
	EXPECT_EQ(*m.getInitialState(), *m.getCurrentState());
	EXPECT_EQ(*m.getInitialState(), *m.getPreviousState());
}

TEST_F(MassSpringMechanicalValidationTests, OneSpringFrequencyTest)
{
	m_numNodes = 2;
	m_totalMass = 0.1;
	m_springStiffness = 1.0;
	m_springDamping = 0.0;
	m_rayleighDampingMass = 0.0;
	m_rayleighDampingStiffness = 0.0;
	m_nodeBoundaryConditions.clear();
	m_nodeBoundaryConditions.push_back(0);

	// Only the Modified Euler Explicit integration conserves the energy exactly
	// (explicit adds energy to the system, implicit removes energy to the system)
	MockMassSpring m("MassSpring", m_poseRandom, m_numNodes, m_nodeBoundaryConditions, m_totalMass,
		m_rayleighDampingMass, m_rayleighDampingStiffness, m_springStiffness, m_springDamping,
		SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT_MODIFIED);

	// Pull on the free mass, by simply making the initial length shorter (creating an extension right away)
	std::static_pointer_cast<LinearSpring>(m.getSpring(0))->setRestLength(0.0);
	m.setIsGravityEnabled(false);

	m.initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m.wakeUp();

	// Frequency = 1/(2PI) sqrt(k / m)
	double f = 1.0/(2.0 * M_PI) * sqrt(m_springStiffness / (m_totalMass/2.0));
	// Period = 1/f
	double period = 1.0 / f;
	// Let's look for a time step, factor of period, small enough for stability of Explicit Euler (~1Khz)
	m_dt = period;
	while(m_dt > 1e-3)
	{
		m_dt *= 0.5;
	}

	// Simulate a single mass connected to a spring at the same time for comparison purpose
	Vector3d x0 = SurgSim::Math::getSubVector(m.getInitialState()->getPositions(), 1, 3);
	Vector3d x = x0;
	Vector3d v = Vector3d::Zero();

	double time = 0.0;
	// Let's do all iterations (except the last 2) testing that the mass is NOT back yet to its original position
	while(time < period - 2.0 * m_dt)
	{
		m.beforeUpdate(m_dt);
		m.update(m_dt);
		m.afterUpdate(m_dt);

		// Manually simulate a single mass connected to a spring
		Vector3d anchor = SurgSim::Math::getSubVector(m.getInitialState()->getPositions(), 0, 3);
		Vector3d f = (m_springStiffness * (anchor - x)) / (m_totalMass/2.0);
		v += f * m_dt;
		x += v * m_dt;

		const Vector3d& finalPosition = SurgSim::Math::getSubVector(m.getFinalState()->getPositions(), 1, 3);
		const Vector3d& finalVelocity = SurgSim::Math::getSubVector(m.getFinalState()->getVelocities(), 1, 3);
		const Vector3d& currentPosition = SurgSim::Math::getSubVector(m.getCurrentState()->getPositions(), 1, 3);
		EXPECT_TRUE(finalPosition.isApprox(currentPosition));
		EXPECT_TRUE(finalPosition.isApprox(x, 1e-8));
		EXPECT_TRUE(finalVelocity.isApprox(v, 1e-8));

		Vector3d deltaCompare = finalPosition - x0;
		EXPECT_FALSE(deltaCompare.isZero(1e-11)) << "Error is " << deltaCompare.norm();
		deltaCompare = currentPosition - x0;
		EXPECT_FALSE(deltaCompare.isZero(1e-11)) << "Error is " << deltaCompare.norm();

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

	const Vector3d& finalPosition = SurgSim::Math::getSubVector(m.getFinalState()->getPositions(), 1, 3);
	const Vector3d& currentPosition = SurgSim::Math::getSubVector(m.getCurrentState()->getPositions(), 1, 3);
	Vector3d deltaCompare = finalPosition - x0;
	EXPECT_TRUE(deltaCompare.isZero(2e-9)) << "Error is " << deltaCompare.norm();
	deltaCompare = currentPosition - x0;
	EXPECT_TRUE(deltaCompare.isZero(2e-9)) << "Error is " << deltaCompare.norm();
}

TEST_F(MassSpringMechanicalValidationTests, FallingTest)
{
	// No boundary conditions to let the model fall
	m_nodeBoundaryConditions.clear();

	MockMassSpring m("MassSpring", m_poseRandom, m_numNodes, m_nodeBoundaryConditions, m_totalMass,
		m_rayleighDampingMass, m_rayleighDampingStiffness, m_springStiffness, m_springDamping,
		SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT);

	m.initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m.wakeUp();

	// run few iterations of simulation...
	for (int i = 0; i< 5; i++)
	{
		m.beforeUpdate(m_dt);
		m.update(m_dt);
		m.afterUpdate(m_dt);

		const Vector& v = m.getFinalState()->getVelocities();

		// Making sure that each mass has a velocity directed toward the gravity vector direction
		// with no orthogonal components
		for (size_t nodeId = 0; nodeId < m.getNumMasses(); nodeId++)
		{
			const Vector3d& vi = SurgSim::Math::getSubVector(v, nodeId, 3);
			double vi_dot_g = vi.dot(m.getGravityVector());
			EXPECT_GE(vi_dot_g, 0.0) << "v = " << vi.transpose() << " ; g = " << m.getGravityVector().transpose();
			const Vector3d vi_cross_g = vi.cross(m.getGravityVector());
			EXPECT_TRUE(vi_cross_g.isZero()) << "v = " << vi.transpose() << " ; g = " <<
				m.getGravityVector().transpose() << " vi^g = " << vi_cross_g.transpose();
		}
	}
}

TEST_F(MassSpringMechanicalValidationTests, EnergyTest)
{
	{
		SCOPED_TRACE("Testing energy increase for Euler explicit");

		// For Euler explicit, the energy should increase
		runEnergyTest(SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT, 1);
	}

	{
		SCOPED_TRACE("Testing energy decrease for Euler implicit");

		// For Euler implicit, the energy should decrease
		runEnergyTest(SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT, -1);
	}

	{
		SCOPED_TRACE("Testing energy stability for Modified Euler explicit");

		// For modified Euler explicit , the energy should be stable
		runEnergyTest(SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT_MODIFIED, 0);
	}
}

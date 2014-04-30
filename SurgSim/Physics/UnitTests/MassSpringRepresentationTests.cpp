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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"


using SurgSim::Framework::Runtime;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector;
using SurgSim::Physics::MassSpringRepresentation;
using SurgSim::Physics::LinearSpring;
using SurgSim::Physics::DeformableRepresentationState;

namespace
{
	const double epsilon = 1e-10;
};

class MockMassSpring : public MassSpringRepresentation
{
public:
	explicit MockMassSpring(const std::string& name,
		const SurgSim::Math::RigidTransform3d& pose,
		unsigned int numNodes, std::vector<unsigned int> boundaryConditions,
		double totalMass,
		double rayleighDampingMass, double rayleighDampingStiffness,
		double springStiffness, double springDamping,
		SurgSim::Math::IntegrationScheme integrationScheme) :
		MassSpringRepresentation(name)
	{
		using SurgSim::Math::getSubVector;
		using SurgSim::Math::setSubVector;
		using SurgSim::Physics::Mass;
		using SurgSim::Physics::LinearSpring;

		// Note: setLocalPose MUST be called before WakeUp to be effective !
		setLocalPose(pose);

		std::shared_ptr<DeformableRepresentationState> state;
		state = std::make_shared<DeformableRepresentationState>();
		state->setNumDof(3, numNodes);
		for (unsigned int i = 0; i < numNodes; i++)
		{
			Vector3d p(static_cast<double>(i)/static_cast<double>(numNodes), 0, 0);
			setSubVector(p, i, 3, &state->getPositions());
			addMass(std::make_shared<Mass>(totalMass / numNodes));
		}
		for (auto bc = std::begin(boundaryConditions); bc != std::end(boundaryConditions); bc++)
		{
			state->addBoundaryCondition(*bc);
		}
		for (unsigned int i = 0; i < numNodes - 1; i++)
		{
			std::shared_ptr<LinearSpring> spring = std::make_shared<LinearSpring>(i, i+1);
			spring->setDamping(springDamping);
			spring->setStiffness(springStiffness);
			const Vector3d& xi = getSubVector(state->getPositions(), i, 3);
			const Vector3d& xj = getSubVector(state->getPositions(), i+1, 3);
			spring->setRestLength( (xj - xi).norm() );
			addSpring(spring);
		}
		setInitialState(state);
		setIntegrationScheme(integrationScheme);
		setRayleighDampingMass(rayleighDampingMass);
		setRayleighDampingStiffness(rayleighDampingStiffness);
	}

	virtual ~MockMassSpring()
	{}

	const Vector3d& getGravityVector() const { return getGravity(); }
};

class MassSpringRepresentationTests : public ::testing::Test
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
		m_boundaryConditions.push_back(0);
		m_boundaryConditions.push_back(1);
		m_boundaryConditions.push_back(2);

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

	void FallingTest(MockMassSpring *m);

protected:
	/// Simulation parameters
	double m_dt;

	/// Number of nodes
	unsigned int m_numNodes;
	/// Boundary conditions
	std::vector<unsigned int> m_boundaryConditions;
	/// Total mass (in Kg)
	double m_totalMass;
	/// Spring stiffness and damping
	double m_springStiffness, m_springDamping;
	/// Rayleigh damping (mass and stiffness coefs)
	double m_rayleighDampingMass, m_rayleighDampingStiffness;

	/// IdentityPose and random pose
	SurgSim::Math::RigidTransform3d m_poseIdentity, m_poseRandom;
};

TEST_F(MassSpringRepresentationTests, Constructor)
{
	ASSERT_NO_THROW({MassSpringRepresentation m("MassSpring");});

	ASSERT_NO_THROW({MassSpringRepresentation* m = new MassSpringRepresentation("MassSpring"); delete m;});

	ASSERT_NO_THROW({std::shared_ptr<MassSpringRepresentation> m = \
		std::make_shared<MassSpringRepresentation>("MassSpring");});
}

TEST_F(MassSpringRepresentationTests, SetGetMethods)
{
	using SurgSim::Math::setSubVector;
	using SurgSim::Physics::Mass;
	using SurgSim::Physics::LinearSpring;

	std::string name = "MassSpring";
	MassSpringRepresentation m(name);

	EXPECT_EQ(name, m.getName());

	// set/get InitialPose
	m.setLocalPose(m_poseRandom);
	EXPECT_TRUE(m.getLocalPose().isApprox(m_poseRandom));

	EXPECT_TRUE(m.getPose().isApprox(m_poseRandom));

	// get{NumDof | NumMasses | NumSprings} initial value is 0
	EXPECT_EQ(0u, m.getNumDof());
	EXPECT_EQ(0u, m.getNumMasses());
	EXPECT_EQ(0u, m.getNumSprings());

	// setInitialState is part of DeformableRepresentation...already tested !
	std::shared_ptr<DeformableRepresentationState> state;
	state = std::make_shared<DeformableRepresentationState>();
	state->setNumDof(3, 2);
	state->getPositions().setRandom();
	m.setInitialState(state);

	// addMass/getNumMasses/getMass
	std::shared_ptr<Mass> mass0 = std::make_shared<Mass>(1.0);
	m.addMass(mass0);
	std::shared_ptr<Mass> mass1 = std::make_shared<Mass>(1.1);
	m.addMass(mass1);
	EXPECT_EQ(2u, m.getNumMasses());
	EXPECT_EQ(mass0, m.getMass(0));
	EXPECT_EQ(*mass0, *m.getMass(0));
	EXPECT_EQ(mass1, m.getMass(1));
	EXPECT_EQ(*mass1, *m.getMass(1));

	// addSpring/getNumSprings/getSpring
	std::shared_ptr<LinearSpring> spring0 = std::make_shared<LinearSpring>(0, 1);
	spring0->setStiffness(1.0);
	spring0->setDamping(1.0);
	spring0->setRestLength(1.0);
	m.addSpring(spring0);
	EXPECT_EQ(1u, m.getNumSprings());
	EXPECT_EQ(spring0, m.getSpring(0));
	EXPECT_EQ(*spring0, *m.getSpring(0));

	// getTotalMass
	EXPECT_DOUBLE_EQ(1.0 + 1.1, m.getTotalMass());

	// set/get RayleighDamping{Mass|Stiffness}
	m.setRayleighDampingMass(5.5);
	EXPECT_DOUBLE_EQ(5.5, m.getRayleighDampingMass());
	m.setRayleighDampingStiffness(5.4);
	EXPECT_DOUBLE_EQ(5.4, m.getRayleighDampingStiffness());

	// set/get Type
	EXPECT_EQ(SurgSim::Physics::REPRESENTATION_TYPE_MASSSPRING, m.getType());
}

TEST_F(MassSpringRepresentationTests, NoGravityTest)
{
	// Note the use of identity pose to avoid small variation between the spring rest length and current length
	MockMassSpring m("MassSpring", m_poseIdentity, m_numNodes, m_boundaryConditions, m_totalMass,
		m_rayleighDampingMass, m_rayleighDampingStiffness, m_springStiffness, m_springDamping,
		SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER);

	m.setIsGravityEnabled(false);
	m.initialize(std::make_shared<Runtime>());
	m.wakeUp();

	m.beforeUpdate(m_dt);
	m.update(m_dt);
	m.afterUpdate(m_dt);

	EXPECT_EQ(*m.getInitialState(), *m.getFinalState());
	EXPECT_EQ(*m.getInitialState(), *m.getCurrentState());
	EXPECT_EQ(*m.getInitialState(), *m.getPreviousState());
}

TEST_F(MassSpringRepresentationTests, OneSpringFrequencyTest)
{
	m_numNodes = 2;
	m_totalMass = 0.1;
	m_springStiffness = 1.0;
	m_springDamping = 0.0;
	m_rayleighDampingMass = 0.0;
	m_rayleighDampingStiffness = 0.0;
	m_boundaryConditions.clear();
	m_boundaryConditions.push_back(0);
	m_boundaryConditions.push_back(1);
	m_boundaryConditions.push_back(2);

	// Only the Modified Euler Explicit integration conserves the energy exactly
	// (explicit adds energy to the system, implicit removes energy to the system)
	MockMassSpring m("MassSpring", m_poseRandom, m_numNodes, m_boundaryConditions, m_totalMass,
		m_rayleighDampingMass, m_rayleighDampingStiffness, m_springStiffness, m_springDamping,
		SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER);

	// Pull on the free mass, by simply making the initial length shorter (creating an extension right away)
	std::static_pointer_cast<LinearSpring>(m.getSpring(0))->setRestLength(0.0);
	m.setIsGravityEnabled(false);

	m.initialize(std::make_shared<Runtime>());
	m.wakeUp();

	// Frequency = 1/(2PI) sqrt(k / m)
	double f = 1.0/(2.0 * M_PI) * sqrt(m_springStiffness / (m_totalMass/2.0));
	// Period = 1/f
	double period = 1.0 / f;
	// Let's look for a time step, factor of period, small enough for stability of Explicit Euler (~1Khz)
	m_dt = period;
	while(m_dt > 1e-3) m_dt *= 0.5;

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

TEST_F(MassSpringRepresentationTests, FallingTest)
{
	// No boundary conditions to let the model fall
	m_boundaryConditions.clear();

	MockMassSpring m("MassSpring", m_poseRandom, m_numNodes, m_boundaryConditions, m_totalMass,
		m_rayleighDampingMass, m_rayleighDampingStiffness, m_springStiffness, m_springDamping,
		SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER);

	m.initialize(std::make_shared<Runtime>());
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
		for (unsigned int nodeId = 0; nodeId < m.getNumMasses(); nodeId++)
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

TEST_F(MassSpringRepresentationTests, EnergyTest)
{
	m_dt = 1e-3;

	m_numNodes = 2;
	m_totalMass = 0.1;
	m_springStiffness = 1.0;
	m_springDamping = 0.0;
	m_rayleighDampingMass = 0.0;
	m_rayleighDampingStiffness = 0.0;
	m_boundaryConditions.clear();
	m_boundaryConditions.push_back(0);
	m_boundaryConditions.push_back(1);
	m_boundaryConditions.push_back(2);

	{
		SCOPED_TRACE("Testing energy increase for Euler explicit");

		MockMassSpring m("MassSpring", m_poseRandom, m_numNodes, m_boundaryConditions, m_totalMass,
			m_rayleighDampingMass, m_rayleighDampingStiffness, m_springStiffness, m_springDamping,
			SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER);

		m.initialize(std::make_shared<Runtime>());
		m.wakeUp();

		// Pull on the free mass, by simply making the initial length shorter (creating an extension right away)
		std::static_pointer_cast<LinearSpring>(m.getSpring(0))->setRestLength(0.0);
		m.setIsGravityEnabled(false);

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
			EXPECT_GT(currentEnergy, previousEnergy);

			time += m_dt;
		}
	}

	{
		SCOPED_TRACE("Testing energy decrease for Euler implicit");

		MockMassSpring m("MassSpring", m_poseRandom, m_numNodes, m_boundaryConditions, m_totalMass,
			m_rayleighDampingMass, m_rayleighDampingStiffness, m_springStiffness, m_springDamping,
			SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

		m.initialize(std::make_shared<Runtime>());
		m.wakeUp();

		// Pull on the free mass, by simply making the initial length shorter (creating an extension right away)
		std::static_pointer_cast<LinearSpring>(m.getSpring(0))->setRestLength(0.0);
		m.setIsGravityEnabled(false);

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
			EXPECT_LT(currentEnergy, previousEnergy);

			time += m_dt;
		}
	}

	{
		SCOPED_TRACE("Testing energy stability for Modified Euler explicit");

		MockMassSpring m("MassSpring", m_poseRandom, m_numNodes, m_boundaryConditions, m_totalMass,
			m_rayleighDampingMass, m_rayleighDampingStiffness, m_springStiffness, m_springDamping,
			SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER);

		m.initialize(std::make_shared<Runtime>());
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
			EXPECT_NEAR(currentEnergy, previousEnergy, 2.6e-6) << " Local diff = " << currentEnergy - previousEnergy;
			EXPECT_NEAR(currentEnergy, initialEnergy, 3e-4) << " Global diff = " << currentEnergy - initialEnergy;

			time += m_dt;
		}
	}
}

TEST_F(MassSpringRepresentationTests, ApplyCorrectionTest)
{
	MockMassSpring m("MassSpring", m_poseIdentity, m_numNodes, m_boundaryConditions, m_totalMass,
		m_rayleighDampingMass, m_rayleighDampingStiffness, m_springStiffness, m_springDamping,
		SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER);

	m.initialize(std::make_shared<Runtime>());
	m.wakeUp();

	SurgSim::Math::Vector dv;
	dv.resize(m.getNumDof());
	for (unsigned int i = 0; i < m.getNumDof(); i++)
	{
		dv(i) = static_cast<double>(i);
	}

	Eigen::VectorXd previousX = m.getCurrentState()->getPositions();
	Eigen::VectorXd previousV = m.getCurrentState()->getVelocities();

	m.applyCorrection(m_dt, dv.segment(0, m.getNumDof()));
	Eigen::VectorXd nextX = m.getCurrentState()->getPositions();
	Eigen::VectorXd nextV = m.getCurrentState()->getVelocities();

	EXPECT_TRUE(nextX.isApprox(previousX + dv * m_dt, epsilon));
	EXPECT_TRUE(nextV.isApprox(previousV + dv, epsilon));

	dv(0) = std::numeric_limits<double>::infinity();
	EXPECT_TRUE(m.isActive());
	m.applyCorrection(m_dt, dv.segment(0, m.getNumDof()));
	EXPECT_FALSE(m.isActive());
}

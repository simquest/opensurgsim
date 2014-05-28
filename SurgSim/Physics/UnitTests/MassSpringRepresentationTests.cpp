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
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::Physics::MassSpringRepresentation;
using SurgSim::Physics::MockSpring;

namespace
{
const double epsilon = 1e-10;
};

class MassSpringRepresentationTests : public ::testing::Test
{
public:
	double m_dt;
	double m_rayleighDampingMassParameter;
	double m_rayleighDampingStiffnessParameter;

	Vector m_expectedSpringsForce;
	Vector m_expectedRayleighDampingForce;
	Vector m_expectedGravityForce;
	Matrix m_expectedMass;
	Matrix m_expectedDamping;
	Matrix m_expectedRayleighDamping;
	Matrix m_expectedStiffness;

	std::shared_ptr<MassSpringRepresentation> m_massSpring;
	std::shared_ptr<SurgSim::Math::OdeState> m_initialState;

protected:
	virtual void SetUp() override
	{
		m_dt = 1e-3;
		m_rayleighDampingMassParameter = 0.452;
		m_rayleighDampingStiffnessParameter = 0.242;

		m_massSpring = std::make_shared<MassSpringRepresentation>("MassSpring");

		m_initialState = std::make_shared<SurgSim::Math::OdeState>();
		m_initialState->setNumDof(m_massSpring->getNumDofPerNode(), 3);
		m_initialState->getVelocities().setOnes(); // v = (1...1) to test damping
		m_massSpring->setInitialState(m_initialState);

		std::shared_ptr<MockSpring> element01 = std::make_shared<MockSpring>();
		element01->addNode(0);
		element01->addNode(1);
		m_massSpring->addSpring(element01);

		std::shared_ptr<MockSpring> element12 = std::make_shared<MockSpring>();
		element12->addNode(1);
		element12->addNode(2);
		m_massSpring->addSpring(element12);

		m_massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(1.0));
		m_massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(1.0));
		m_massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(1.0));

		// Mass should be a identity matrix (all masses are 1)
		m_expectedMass = Matrix::Identity(9, 9);

		// Damping should be a diagonal matrix with diagonal (2 2 2 4 4 4 2 2 2)
		m_expectedDamping = Matrix::Zero(9, 9);
		m_expectedDamping.diagonal() << 2.0, 2.0, 2.0, 4.0, 4.0, 4.0, 2.0, 2.0, 2.0;

		// Stiffness should be a diagonal matrix with diagonal (3 3 3 6 6 6 3 3 3)
		m_expectedStiffness = Matrix::Zero(9, 9);
		m_expectedStiffness.diagonal() << 3.0, 3.0, 3.0, 6.0, 6.0, 6.0, 3.0, 3.0, 3.0;

		// Rayleigh damping = alpha.M + beta.K with M and K are diagonals, so the resulting matrix is too
		m_expectedRayleighDamping = m_expectedMass * m_rayleighDampingMassParameter;
		m_expectedRayleighDamping += m_expectedStiffness * m_rayleighDampingStiffnessParameter;

		// FemElements force should be (1 2 3 4 5 6 0 0 0) + (0 0 0 1 2 3 4 5 6) = (1 2 3 5 7 9 4 5 6)
		m_expectedSpringsForce.resize(9);
		m_expectedSpringsForce << 1.0, 2.0, 3.0, 5.0, 7.0, 9.0, 4.0, 5.0, 6.0;

		// Gravity force should be m.gravity for each node of each element
		m_expectedGravityForce = Vector::Zero(9);
		SurgSim::Math::Vector3d g(0.0, -9.81, 0.0);
		m_expectedGravityForce .segment<3>(0) += g; // Mass = 1 for each node
		m_expectedGravityForce .segment<3>(3) += g; // Mass = 1 for each node
		m_expectedGravityForce .segment<3>(6) += g; // Mass = 1 for each node

		// Rayleigh damping force should be -(alpha.M + beta.K).(1...1)^t
		// with (alpha.M + beta.K) a diagonal matrix
		m_expectedRayleighDampingForce = -m_expectedRayleighDamping.diagonal();
	}
};

TEST_F(MassSpringRepresentationTests, Constructor)
{
	ASSERT_NO_THROW({MassSpringRepresentation m("MassSpring");});

	ASSERT_NO_THROW({MassSpringRepresentation* m = new MassSpringRepresentation("MassSpring"); delete m;});

	ASSERT_NO_THROW({std::shared_ptr<MassSpringRepresentation> m =
		std::make_shared<MassSpringRepresentation>("MassSpring");});
}

TEST_F(MassSpringRepresentationTests, SetGetMethods)
{
	using SurgSim::Math::setSubVector;
	using SurgSim::Physics::Mass;
	using SurgSim::Physics::LinearSpring;

	MassSpringRepresentation m("MassSpring");

	// set/get InitialPose
	SurgSim::Math::RigidTransform3d poseRandom;
	SurgSim::Math::Quaterniond q(1.0, 2.0, 3.0, 4.0);
	SurgSim::Math::Vector3d t(1.0, 2.0, 3.0);
	q.normalize();
	poseRandom = SurgSim::Math::makeRigidTransform(q, t);
	m.setLocalPose(poseRandom);
	EXPECT_TRUE(m.getLocalPose().isApprox(poseRandom));

	EXPECT_TRUE(m.getPose().isApprox(poseRandom));

	// get{NumDof | NumMasses | NumSprings} initial value is 0
	EXPECT_EQ(0u, m.getNumDof());
	EXPECT_EQ(0u, m.getNumMasses());
	EXPECT_EQ(0u, m.getNumSprings());

	// setInitialState is part of DeformableRepresentation...already tested !
	std::shared_ptr<SurgSim::Math::OdeState> state = std::make_shared<SurgSim::Math::OdeState>();
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

TEST_F(MassSpringRepresentationTests, BeforeUpdateTest)
{
	const double dt = 1e-3;

	MassSpringRepresentation m("MassSpring");

	// Missing initial state, masses and springs
	EXPECT_THROW(m.beforeUpdate(dt), SurgSim::Framework::AssertionFailure);
	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(m.getNumDofPerNode(), 3);
	m.setInitialState(initialState);

	// Missing springs and 2 masses
	EXPECT_THROW(m.beforeUpdate(dt), SurgSim::Framework::AssertionFailure);
	m.addSpring(std::make_shared<SurgSim::Physics::LinearSpring>(0, 1));

	// Missing 3 masses
	EXPECT_THROW(m.beforeUpdate(dt), SurgSim::Framework::AssertionFailure);
	m.addMass(std::make_shared<SurgSim::Physics::Mass>(1.2));

	// Missing 2 masses
	EXPECT_THROW(m.beforeUpdate(dt), SurgSim::Framework::AssertionFailure);
	m.addMass(std::make_shared<SurgSim::Physics::Mass>(1.2));

	// Missing 1 mass
	EXPECT_THROW(m.beforeUpdate(dt), SurgSim::Framework::AssertionFailure);
	m.addMass(std::make_shared<SurgSim::Physics::Mass>(1.2));

	EXPECT_NO_THROW(m.beforeUpdate(dt));
}

TEST_F(MassSpringRepresentationTests, AfterUpdateTest)
{
	const double dt = 1e-3;

	{
		SCOPED_TRACE("Valid state");
		MassSpringRepresentation m("MassSpring");
		std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
		initialState->setNumDof(m.getNumDofPerNode(), 2);
		m.setInitialState(initialState);

		m.addSpring(std::make_shared<SurgSim::Physics::LinearSpring>(0, 1));
		m.addMass(std::make_shared<SurgSim::Physics::Mass>(1.2));
		m.addMass(std::make_shared<SurgSim::Physics::Mass>(1.2));

		EXPECT_TRUE(m.isActive());
		EXPECT_NO_THROW(m.afterUpdate(dt));
		EXPECT_TRUE(m.isActive());
	}

	{
		SCOPED_TRACE("Invalid state");
		MassSpringRepresentation m("MassSpring");
		std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
		initialState->setNumDof(m.getNumDofPerNode(), 2);
		initialState->getPositions()[0] = std::numeric_limits<double>::infinity();
		m.setInitialState(initialState);

		m.addSpring(std::make_shared<SurgSim::Physics::LinearSpring>(0, 1));
		m.addMass(std::make_shared<SurgSim::Physics::Mass>(1.2));
		m.addMass(std::make_shared<SurgSim::Physics::Mass>(1.2));

		EXPECT_TRUE(m.isActive());
		EXPECT_NO_THROW(m.afterUpdate(dt));
		EXPECT_FALSE(m.isActive());
	}
}

TEST_F(MassSpringRepresentationTests, TransformInitialStateTest)
{
	using SurgSim::Math::Vector;

	std::shared_ptr<MassSpringRepresentation> massSpring = std::make_shared<MassSpringRepresentation>("MassSpring");

	const size_t numNodes = 2;
	const size_t numDofPerNode = massSpring->getNumDofPerNode();
	const size_t numDof = numDofPerNode * numNodes;

	SurgSim::Math::RigidTransform3d initialPose;
	SurgSim::Math::Quaterniond q(1.0, 2.0, 3.0, 4.0);
	SurgSim::Math::Vector3d t(1.0, 2.0, 3.0);
	q.normalize();
	initialPose = SurgSim::Math::makeRigidTransform(q, t);
	massSpring->setLocalPose(initialPose);

	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(numDofPerNode, numNodes);
	Vector x = Vector::LinSpaced(numDof, 1.0, static_cast<double>(numDof));
	Vector v = Vector::Ones(numDof);
	initialState->getPositions() = x;
	initialState->getVelocities() = v;
	massSpring->setInitialState(initialState);

	Vector expectedX = x, expectedV = v;
	for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
	{
		expectedX.segment<3>(numDofPerNode * nodeId) = initialPose * x.segment<3>(numDofPerNode * nodeId);
		expectedV.segment<3>(numDofPerNode * nodeId) = initialPose.linear() * v.segment<3>(numDofPerNode * nodeId);
	}

	// Initialize the component
	ASSERT_TRUE(massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>()));
	// Wake-up the component => apply the pose to the initial state
	ASSERT_TRUE(massSpring->wakeUp());

	EXPECT_TRUE(massSpring->getInitialState()->getPositions().isApprox(expectedX));
	EXPECT_TRUE(massSpring->getInitialState()->getVelocities().isApprox(expectedV));
}

TEST_F(MassSpringRepresentationTests, ComputesWithNoGravityAndNoDampingTest)
{
	using SurgSim::Math::Vector3d;

	Vector *F;
	Matrix *M, *D, *K;

	// No gravity, no Rayleigh damping
	// computeF tests addFemElementsForce
	m_massSpring->setIsGravityEnabled(false);
	m_massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_massSpring->wakeUp();

	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeF(*m_initialState).isApprox(m_expectedSpringsForce)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeM(*m_initialState).isApprox(m_expectedMass)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeD(*m_initialState).isApprox(m_expectedDamping)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeK(*m_initialState).isApprox(m_expectedStiffness)));
	// Test combo method computeFMDK
	EXPECT_NO_THROW(m_massSpring->computeFMDK(*m_initialState, &F, &M, &D, &K));
	EXPECT_TRUE((*F).isApprox(m_expectedSpringsForce));
	EXPECT_TRUE((*M).isApprox(m_expectedMass));
	EXPECT_TRUE((*D).isApprox(m_expectedDamping));
	EXPECT_TRUE((*K).isApprox(m_expectedStiffness));
}

TEST_F(MassSpringRepresentationTests, ComputesWithNoGravityAndDampingTest)
{
	using SurgSim::Math::Vector3d;

	Vector *F;
	Matrix *M, *D, *K;

	// No gravity, Rayleigh damping
	// computeF tests addFemElementsForce and addRayleighDampingForce
	m_massSpring->setIsGravityEnabled(false);
	m_massSpring->setRayleighDampingMass(m_rayleighDampingMassParameter);
	m_massSpring->setRayleighDampingStiffness(m_rayleighDampingStiffnessParameter);
	m_massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_massSpring->wakeUp();

	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeF(*m_initialState).isApprox(
		m_expectedSpringsForce + m_expectedRayleighDampingForce)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeM(*m_initialState).isApprox(m_expectedMass)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeD(*m_initialState).isApprox(
		m_expectedDamping + m_expectedRayleighDamping)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeK(*m_initialState).isApprox(m_expectedStiffness)));
	// Test combo method computeFMDK
	EXPECT_NO_THROW(m_massSpring->computeFMDK(*m_initialState, &F, &M, &D, &K));
	EXPECT_TRUE((*F).isApprox(m_expectedSpringsForce + m_expectedRayleighDampingForce));
	EXPECT_TRUE((*M).isApprox(m_expectedMass));
	EXPECT_TRUE((*D).isApprox(m_expectedDamping + m_expectedRayleighDamping));
	EXPECT_TRUE((*K).isApprox(m_expectedStiffness));
}

TEST_F(MassSpringRepresentationTests, ComputesWithGravityAndNoDampingTest)
{
	using SurgSim::Math::Vector3d;

	Vector *F;
	Matrix *M, *D, *K;

	// Gravity, no Rayleigh damping
	// computeF tests addFemElementsForce and addGravityForce
	m_massSpring->setIsGravityEnabled(true);
	m_massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_massSpring->wakeUp();

	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeF(*m_initialState).isApprox(
		m_expectedSpringsForce + m_expectedGravityForce)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeM(*m_initialState).isApprox(m_expectedMass)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeD(*m_initialState).isApprox(m_expectedDamping)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeK(*m_initialState).isApprox(m_expectedStiffness)));
	// Test combo method computeFMDK
	EXPECT_NO_THROW(m_massSpring->computeFMDK(*m_initialState, &F, &M, &D, &K));
	EXPECT_TRUE((*F).isApprox(m_expectedSpringsForce + m_expectedGravityForce));
	EXPECT_TRUE((*M).isApprox(m_expectedMass));
	EXPECT_TRUE((*D).isApprox(m_expectedDamping));
	EXPECT_TRUE((*K).isApprox(m_expectedStiffness));
}

TEST_F(MassSpringRepresentationTests, ComputesWithGravityAndDampingTest)
{
	using SurgSim::Math::Vector3d;

	Vector *F;
	Matrix *M, *D, *K;

	// Gravity, Rayleigh damping
	// computeF tests addFemElementsForce, addRayleighDampingForce and addGravityForce
	m_massSpring->setIsGravityEnabled(true);
	m_massSpring->setRayleighDampingMass(m_rayleighDampingMassParameter);
	m_massSpring->setRayleighDampingStiffness(m_rayleighDampingStiffnessParameter);
	m_massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_massSpring->wakeUp();

	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeF(*m_initialState).isApprox(
		m_expectedSpringsForce + m_expectedRayleighDampingForce + m_expectedGravityForce)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeM(*m_initialState).isApprox(m_expectedMass)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeD(*m_initialState).isApprox(
		m_expectedDamping + m_expectedRayleighDamping)));
	EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeK(*m_initialState).isApprox(m_expectedStiffness)));
	// Test combo method computeFMDK
	EXPECT_NO_THROW(m_massSpring->computeFMDK(*m_initialState, &F, &M, &D, &K));
	EXPECT_TRUE((*F).isApprox(m_expectedSpringsForce + m_expectedRayleighDampingForce + m_expectedGravityForce));
	EXPECT_TRUE((*M).isApprox(m_expectedMass));
	EXPECT_TRUE((*D).isApprox(m_expectedDamping + m_expectedRayleighDamping));
	EXPECT_TRUE((*K).isApprox(m_expectedStiffness));
}

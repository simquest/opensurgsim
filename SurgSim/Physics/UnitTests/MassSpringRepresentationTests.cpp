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
#include "SurgSim/Physics/MassSpringLocalization.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::Math::Vector;
using SurgSim::Math::Matrix;
using SurgSim::Physics::MassSpringRepresentation;
using SurgSim::Physics::MassSpringLocalization;
using SurgSim::Physics::MockLocalization;
using SurgSim::Physics::MockMassSpring;
using SurgSim::Physics::MockSpring;


namespace
{
const double epsilon = 1e-10;
};

class MassSpringRepresentationTests : public ::testing::Test
{
protected:
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

	std::shared_ptr<MockMassSpring> m_massSpring;
	std::shared_ptr<SurgSim::Math::OdeState> m_initialState;
	std::shared_ptr<SurgSim::Physics::MassSpringLocalization> m_localization;

	void SetUp() override
	{
		m_massSpring = std::make_shared<MockMassSpring>();

		m_dt = 1e-3;
		m_rayleighDampingMassParameter = 0.452;
		m_rayleighDampingStiffnessParameter = 0.242;

		m_initialState = std::make_shared<SurgSim::Math::OdeState>();
		m_initialState->setNumDof(m_massSpring->getNumDofPerNode(), 3);
		m_initialState->getVelocities().setOnes(); // v = (1...1) to test damping
		m_massSpring->setInitialState(m_initialState);

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

	void addSprings()
	{
		std::shared_ptr<MockSpring> element01 = std::make_shared<MockSpring>();
		element01->addNode(0);
		element01->addNode(1);
		m_massSpring->addSpring(element01);

		std::shared_ptr<MockSpring> element12 = std::make_shared<MockSpring>();
		element12->addNode(1);
		element12->addNode(2);
		m_massSpring->addSpring(element12);
	}

	void addMasses()
	{
		m_massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(1.0));
		m_massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(1.0));
		m_massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(1.0));
	}

	void createLocalization()
	{
		m_localization = std::make_shared<SurgSim::Physics::MassSpringLocalization>();
		m_localization->setRepresentation(m_massSpring);
		m_localization->setLocalNode(0);
	}
};

TEST_F(MassSpringRepresentationTests, Constructor)
{
	ASSERT_NO_THROW({MassSpringRepresentation m("MassSpring");});

	ASSERT_NO_THROW({MassSpringRepresentation* m = new MassSpringRepresentation("MassSpring"); delete m;});

	ASSERT_NO_THROW({std::shared_ptr<MassSpringRepresentation> m =
						 std::make_shared<MassSpringRepresentation>("MassSpring");
					});
}

TEST_F(MassSpringRepresentationTests, SetGetMethods)
{
	using SurgSim::Math::setSubVector;
	using SurgSim::Physics::Mass;
	using SurgSim::Physics::LinearSpring;

	std::shared_ptr<MassSpringRepresentation> massSpring = std::make_shared<MassSpringRepresentation>("MassSpring");

	// set/get InitialPose
	SurgSim::Math::RigidTransform3d poseRandom;
	SurgSim::Math::Quaterniond q(1.0, 2.0, 3.0, 4.0);
	SurgSim::Math::Vector3d t(1.0, 2.0, 3.0);
	q.normalize();
	poseRandom = SurgSim::Math::makeRigidTransform(q, t);
	massSpring->setLocalPose(poseRandom);
	EXPECT_TRUE(massSpring->getLocalPose().isApprox(poseRandom));

	EXPECT_TRUE(massSpring->getPose().isApprox(poseRandom));

	// get{NumDof | NumMasses | NumSprings} initial value is 0
	EXPECT_EQ(0u, massSpring->getNumDof());
	EXPECT_EQ(0u, massSpring->getNumMasses());
	EXPECT_EQ(0u, massSpring->getNumSprings());

	// setInitialState is part of DeformableRepresentation...already tested !
	std::shared_ptr<SurgSim::Math::OdeState> state = std::make_shared<SurgSim::Math::OdeState>();
	state->setNumDof(3, 2);
	state->getPositions().setRandom();
	massSpring->setInitialState(state);

	// addMass/getNumMasses/getMass
	std::shared_ptr<Mass> mass0 = std::make_shared<Mass>(1.0);
	massSpring->addMass(mass0);
	std::shared_ptr<Mass> mass1 = std::make_shared<Mass>(1.1);
	massSpring->addMass(mass1);
	EXPECT_EQ(2u, massSpring->getNumMasses());
	EXPECT_EQ(mass0, massSpring->getMass(0));
	EXPECT_EQ(*mass0, *massSpring->getMass(0));
	EXPECT_EQ(mass1, massSpring->getMass(1));
	EXPECT_EQ(*mass1, *massSpring->getMass(1));

	// addSpring/getNumSprings/getSpring
	std::shared_ptr<LinearSpring> spring0 = std::make_shared<LinearSpring>(0, 1);
	spring0->setStiffness(1.0);
	spring0->setDamping(1.0);
	spring0->setRestLength(1.0);
	massSpring->addSpring(spring0);
	EXPECT_EQ(1u, massSpring->getNumSprings());
	EXPECT_EQ(spring0, massSpring->getSpring(0));
	EXPECT_EQ(*spring0, *massSpring->getSpring(0));

	// getTotalMass
	EXPECT_DOUBLE_EQ(1.0 + 1.1, massSpring->getTotalMass());

	// set/get RayleighDamping{Mass|Stiffness}
	massSpring->setRayleighDampingMass(5.5);
	EXPECT_DOUBLE_EQ(5.5, massSpring->getRayleighDampingMass());
	massSpring->setRayleighDampingStiffness(5.4);
	EXPECT_DOUBLE_EQ(5.4, massSpring->getRayleighDampingStiffness());
}

TEST_F(MassSpringRepresentationTests, BeforeUpdateTest)
{
	const double dt = 1e-3;

	std::shared_ptr<MassSpringRepresentation> massSpring = std::make_shared<MassSpringRepresentation>("MassSpring");

	// Missing initial state, masses and springs
	EXPECT_THROW(massSpring->beforeUpdate(dt), SurgSim::Framework::AssertionFailure);
	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(massSpring->getNumDofPerNode(), 3);
	massSpring->setInitialState(initialState);

	// Missing springs and 2 masses
	EXPECT_THROW(massSpring->beforeUpdate(dt), SurgSim::Framework::AssertionFailure);
	massSpring->addSpring(std::make_shared<SurgSim::Physics::LinearSpring>(0, 1));

	// Missing 3 masses
	EXPECT_THROW(massSpring->beforeUpdate(dt), SurgSim::Framework::AssertionFailure);
	massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(1.2));

	// Missing 2 masses
	EXPECT_THROW(massSpring->beforeUpdate(dt), SurgSim::Framework::AssertionFailure);
	massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(1.2));

	// Missing 1 mass
	EXPECT_THROW(massSpring->beforeUpdate(dt), SurgSim::Framework::AssertionFailure);
	massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(1.2));

	EXPECT_NO_THROW(massSpring->beforeUpdate(dt));
}

TEST_F(MassSpringRepresentationTests, TransformInitialStateTest)
{
	using SurgSim::Math::Vector;

	const size_t numNodes = 2;
	const size_t numDofPerNode = m_massSpring->getNumDofPerNode();
	const size_t numDof = numDofPerNode * numNodes;

	SurgSim::Math::RigidTransform3d initialPose;
	SurgSim::Math::Quaterniond q(1.0, 2.0, 3.0, 4.0);
	SurgSim::Math::Vector3d t(1.0, 2.0, 3.0);
	q.normalize();
	initialPose = SurgSim::Math::makeRigidTransform(q, t);
	m_massSpring->setLocalPose(initialPose);

	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(numDofPerNode, numNodes);
	Vector x = Vector::LinSpaced(numDof, 1.0, static_cast<double>(numDof));
	Vector v = Vector::Ones(numDof);
	initialState->getPositions() = x;
	initialState->getVelocities() = v;
	m_massSpring->setInitialState(initialState);

	Vector expectedX = x, expectedV = v;
	for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
	{
		expectedX.segment<3>(numDofPerNode * nodeId) = initialPose * x.segment<3>(numDofPerNode * nodeId);
		expectedV.segment<3>(numDofPerNode * nodeId) = initialPose.linear() * v.segment<3>(numDofPerNode * nodeId);
	}

	// Initialize the component
	ASSERT_TRUE(m_massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>()));
	// Wake-up the component => apply the pose to the initial state
	ASSERT_TRUE(m_massSpring->wakeUp());

	EXPECT_TRUE(m_massSpring->getInitialState()->getPositions().isApprox(expectedX));
	EXPECT_TRUE(m_massSpring->getInitialState()->getVelocities().isApprox(expectedV));
}

TEST_F(MassSpringRepresentationTests, ExternalForceAPITest)
{
	using SurgSim::Math::Vector;
	using SurgSim::Math::SparseMatrix;

	std::shared_ptr<MockMassSpring> massSpring = std::make_shared<MockMassSpring>();
	std::shared_ptr<MassSpringLocalization> localization =
		std::make_shared<MassSpringLocalization>();
	std::shared_ptr<MockLocalization> wrongLocalizationType =
		std::make_shared<MockLocalization>();

	// External force vector not initialized until the initial state has been set (it contains the #dof...)
	EXPECT_EQ(0, massSpring->getExternalForce().size());
	EXPECT_EQ(0, massSpring->getExternalStiffness().size());
	EXPECT_EQ(0, massSpring->getExternalDamping().size());

	massSpring->setInitialState(m_initialState);

	massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(1.0));
	massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(1.0));
	massSpring->addSpring(std::make_shared<SurgSim::Physics::LinearSpring>(0, 1));

	// Vector initialized (properly sized and zeroed)
	SparseMatrix zeroMatrix(static_cast<SparseMatrix::Index>(massSpring->getNumDof()),
							static_cast<SparseMatrix::Index>(massSpring->getNumDof()));
	zeroMatrix.setZero();
	EXPECT_NE(0, massSpring->getExternalForce().size());
	EXPECT_NE(0, massSpring->getExternalStiffness().size());
	EXPECT_NE(0, massSpring->getExternalDamping().size());
	EXPECT_EQ(massSpring->getNumDof(), massSpring->getExternalForce().size());
	EXPECT_EQ(massSpring->getNumDof(), massSpring->getExternalStiffness().cols());
	EXPECT_EQ(massSpring->getNumDof(), massSpring->getExternalStiffness().rows());
	EXPECT_EQ(massSpring->getNumDof(), massSpring->getExternalDamping().cols());
	EXPECT_EQ(massSpring->getNumDof(), massSpring->getExternalDamping().rows());
	EXPECT_TRUE(massSpring->getExternalForce().isZero());
	EXPECT_TRUE(massSpring->getExternalStiffness().isApprox(zeroMatrix));
	EXPECT_TRUE(massSpring->getExternalDamping().isApprox(zeroMatrix));

	localization->setRepresentation(massSpring);
	localization->setLocalNode(0);
	wrongLocalizationType->setRepresentation(massSpring);

	Vector F = Vector::LinSpaced(massSpring->getNumDofPerNode(), -3.12, 4.09);
	Matrix K = Matrix::Ones(massSpring->getNumDofPerNode(), massSpring->getNumDofPerNode()) * 0.34;
	Matrix D = K + Matrix::Identity(massSpring->getNumDofPerNode(), massSpring->getNumDofPerNode());
	Vector expectedF = Vector::Zero(massSpring->getNumDof());
	expectedF.segment(0, massSpring->getNumDofPerNode()) = F;
	Matrix expectedK = Matrix::Zero(massSpring->getNumDof(), massSpring->getNumDof());
	expectedK.block(0, 0, massSpring->getNumDofPerNode(), massSpring->getNumDofPerNode()) = K;
	Matrix expectedD = Matrix::Zero(massSpring->getNumDof(), massSpring->getNumDof());
	expectedD.block(0, 0, massSpring->getNumDofPerNode(), massSpring->getNumDofPerNode()) = D;

	ASSERT_THROW(massSpring->addExternalGeneralizedForce(nullptr, F, K, D),
				 SurgSim::Framework::AssertionFailure);
	ASSERT_THROW(massSpring->addExternalGeneralizedForce(wrongLocalizationType, F, K, D),
				 SurgSim::Framework::AssertionFailure);

	massSpring->addExternalGeneralizedForce(localization, F, K, D);
	EXPECT_FALSE(massSpring->getExternalForce().isZero());
	EXPECT_FALSE(massSpring->getExternalStiffness().isApprox(zeroMatrix));
	EXPECT_FALSE(massSpring->getExternalDamping().isApprox(zeroMatrix));
	EXPECT_TRUE(massSpring->getExternalForce().isApprox(expectedF));
	EXPECT_TRUE(massSpring->getExternalStiffness().isApprox(expectedK));
	EXPECT_TRUE(massSpring->getExternalDamping().isApprox(expectedD));

	massSpring->addExternalGeneralizedForce(localization, F, K, D);
	EXPECT_TRUE(massSpring->getExternalForce().isApprox(2.0 * expectedF));
	EXPECT_TRUE(massSpring->getExternalStiffness().isApprox(2.0 * expectedK));
	EXPECT_TRUE(massSpring->getExternalDamping().isApprox(2.0 * expectedD));
}

TEST_F(MassSpringRepresentationTests, ComputesWithNoGravityAndNoDampingTest)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::SparseMatrix;

	Vector* F;
	SparseMatrix* M, *D, *K;

	addMasses();
	addSprings();
	createLocalization();

	// No gravity, no Rayleigh damping
	// computeF tests addFemElementsForce
	m_massSpring->setIsGravityEnabled(false);
	m_massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_massSpring->wakeUp();

	{
		SCOPED_TRACE("Without external force");

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

	size_t numDofPerNode = m_massSpring->getNumDofPerNode();
	size_t numDof = m_massSpring->getNumDof();

	{
		SCOPED_TRACE("With external force");

		Vector externalLocalForce = Vector::LinSpaced(numDofPerNode, 1.1, 4.56);
		Matrix externalLocalK = Matrix::Ones(numDofPerNode, numDofPerNode);
		Matrix externalLocalD = 3.1 * externalLocalK + Matrix::Identity(numDofPerNode, numDofPerNode);
		Vector externalForce = Vector::Zero(numDof);
		Matrix externalK = Matrix::Zero(numDof, numDof);
		Matrix externalD = Matrix::Zero(numDof, numDof);
		externalForce.segment(0, numDofPerNode) = externalLocalForce;
		externalK.block(0, 0, numDofPerNode, numDofPerNode) = externalLocalK;
		externalD.block(0, 0, numDofPerNode, numDofPerNode) = externalLocalD;
		m_massSpring->addExternalGeneralizedForce(m_localization, externalLocalForce, externalLocalK, externalLocalD);
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeF(*m_initialState).isApprox(
										m_expectedSpringsForce + externalForce)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeK(*m_initialState).isApprox(
										m_expectedStiffness + externalK)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeD(*m_initialState).isApprox(m_expectedDamping + externalD)));
		EXPECT_NO_THROW(m_massSpring->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(m_expectedSpringsForce + externalForce));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness + externalK));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + externalD));
	}
}

TEST_F(MassSpringRepresentationTests, ComputesWithNoGravityAndDampingTest)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::SparseMatrix;

	Vector* F;
	SparseMatrix* M, *D, *K;

	addMasses();
	addSprings();
	createLocalization();

	// No gravity, Rayleigh damping
	// computeF tests addFemElementsForce and addRayleighDampingForce
	m_massSpring->setIsGravityEnabled(false);
	m_massSpring->setRayleighDampingMass(m_rayleighDampingMassParameter);
	m_massSpring->setRayleighDampingStiffness(m_rayleighDampingStiffnessParameter);
	m_massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_massSpring->wakeUp();

	SurgSim::Math::Vector expectedForce = m_expectedSpringsForce + m_expectedRayleighDampingForce;

	{
		SCOPED_TRACE("Without external force");

		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeF(*m_initialState).isApprox(expectedForce)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeM(*m_initialState).isApprox(m_expectedMass)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeD(*m_initialState).isApprox(
										m_expectedDamping + m_expectedRayleighDamping)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeK(*m_initialState).isApprox(m_expectedStiffness)));
		// Test combo method computeFMDK
		EXPECT_NO_THROW(m_massSpring->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedForce));
		EXPECT_TRUE((*M).isApprox(m_expectedMass));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + m_expectedRayleighDamping));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness));
	}

	size_t numDofPerNode = m_massSpring->getNumDofPerNode();
	size_t numDof = m_massSpring->getNumDof();

	{
		SCOPED_TRACE("With external force");

		Vector externalLocalForce = Vector::LinSpaced(numDofPerNode, 1.1, 4.56);
		Matrix externalLocalK = Matrix::Ones(numDofPerNode, numDofPerNode);
		Matrix externalLocalD = 3.1 * externalLocalK + Matrix::Identity(numDofPerNode, numDofPerNode);
		Vector externalForce = Vector::Zero(numDof);
		Matrix externalK = Matrix::Zero(numDof, numDof);
		Matrix externalD = Matrix::Zero(numDof, numDof);
		externalForce.segment(0, numDofPerNode) = externalLocalForce;
		externalK.block(0, 0, numDofPerNode, numDofPerNode) = externalLocalK;
		externalD.block(0, 0, numDofPerNode, numDofPerNode) = externalLocalD;
		m_massSpring->addExternalGeneralizedForce(m_localization, externalLocalForce, externalLocalK, externalLocalD);
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeF(*m_initialState).isApprox(expectedForce + externalForce)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeK(*m_initialState).isApprox(
										m_expectedStiffness + externalK)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeD(*m_initialState).isApprox(
										m_expectedDamping + m_expectedRayleighDamping + externalD)));
		EXPECT_NO_THROW(m_massSpring->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedForce + externalForce));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness + externalK));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + m_expectedRayleighDamping + externalD));
	}
}

TEST_F(MassSpringRepresentationTests, ComputesWithGravityAndNoDampingTest)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::SparseMatrix;

	Vector* F;
	SparseMatrix* M, *D, *K;

	addMasses();
	addSprings();
	createLocalization();

	// Gravity, no Rayleigh damping
	// computeF tests addFemElementsForce and addGravityForce
	m_massSpring->setIsGravityEnabled(true);
	m_massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_massSpring->wakeUp();

	SurgSim::Math::Vector expectedForce = m_expectedSpringsForce + m_expectedGravityForce;

	{
		SCOPED_TRACE("Without external force");

		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeF(*m_initialState).isApprox(expectedForce)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeM(*m_initialState).isApprox(m_expectedMass)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeD(*m_initialState).isApprox(m_expectedDamping)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeK(*m_initialState).isApprox(m_expectedStiffness)));
		// Test combo method computeFMDK
		EXPECT_NO_THROW(m_massSpring->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedForce));
		EXPECT_TRUE((*M).isApprox(m_expectedMass));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness));
	}

	size_t numDofPerNode = m_massSpring->getNumDofPerNode();
	size_t numDof = m_massSpring->getNumDof();

	{
		SCOPED_TRACE("With external force");

		Vector externalLocalForce = Vector::LinSpaced(numDofPerNode, 1.1, 4.56);
		Matrix externalLocalK = Matrix::Ones(numDofPerNode, numDofPerNode);
		Matrix externalLocalD = 3.1 * externalLocalK + Matrix::Identity(numDofPerNode, numDofPerNode);
		Vector externalForce = Vector::Zero(numDof);
		Matrix externalK = Matrix::Zero(numDof, numDof);
		Matrix externalD = Matrix::Zero(numDof, numDof);
		externalForce.segment(0, numDofPerNode) = externalLocalForce;
		externalK.block(0, 0, numDofPerNode, numDofPerNode) = externalLocalK;
		externalD.block(0, 0, numDofPerNode, numDofPerNode) = externalLocalD;
		m_massSpring->addExternalGeneralizedForce(m_localization, externalLocalForce, externalLocalK, externalLocalD);
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeF(*m_initialState).isApprox(expectedForce + externalForce)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeK(*m_initialState).isApprox(
										m_expectedStiffness + externalK)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeD(*m_initialState).isApprox(m_expectedDamping + externalD)));
		EXPECT_NO_THROW(m_massSpring->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedForce + externalForce));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness + externalK));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + externalD));
	}
}

TEST_F(MassSpringRepresentationTests, ComputesWithGravityAndDampingTest)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::SparseMatrix;

	Vector* F;
	SparseMatrix* M, *D, *K;

	addMasses();
	addSprings();
	createLocalization();

	// Gravity, Rayleigh damping
	// computeF tests addFemElementsForce, addRayleighDampingForce and addGravityForce
	m_massSpring->setIsGravityEnabled(true);
	m_massSpring->setRayleighDampingMass(m_rayleighDampingMassParameter);
	m_massSpring->setRayleighDampingStiffness(m_rayleighDampingStiffnessParameter);
	m_massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_massSpring->wakeUp();

	SurgSim::Math::Vector expectedForce = m_expectedSpringsForce + m_expectedRayleighDampingForce +
										  m_expectedGravityForce;

	{
		SCOPED_TRACE("Without external force");

		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeF(*m_initialState).isApprox(expectedForce)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeM(*m_initialState).isApprox(m_expectedMass)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeD(*m_initialState).isApprox(
										m_expectedDamping + m_expectedRayleighDamping)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeK(*m_initialState).isApprox(m_expectedStiffness)));
		// Test combo method computeFMDK
		EXPECT_NO_THROW(m_massSpring->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedForce));
		EXPECT_TRUE((*M).isApprox(m_expectedMass));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + m_expectedRayleighDamping));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness));
	}

	size_t numDofPerNode = m_massSpring->getNumDofPerNode();
	size_t numDof = m_massSpring->getNumDof();

	{
		SCOPED_TRACE("With external force");

		Vector externalLocalForce = Vector::LinSpaced(numDofPerNode, 1.1, 4.56);
		Matrix externalLocalK = Matrix::Ones(numDofPerNode, numDofPerNode);
		Matrix externalLocalD = 3.1 * externalLocalK + Matrix::Identity(numDofPerNode, numDofPerNode);
		Vector externalForce = Vector::Zero(numDof);
		Matrix externalK = Matrix::Zero(numDof, numDof);
		Matrix externalD = Matrix::Zero(numDof, numDof);
		externalForce.segment(0, numDofPerNode) = externalLocalForce;
		externalK.block(0, 0, numDofPerNode, numDofPerNode) = externalLocalK;
		externalD.block(0, 0, numDofPerNode, numDofPerNode) = externalLocalD;
		m_massSpring->addExternalGeneralizedForce(m_localization, externalLocalForce, externalLocalK, externalLocalD);
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeF(*m_initialState).isApprox(expectedForce + externalForce)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeK(*m_initialState).isApprox(
										m_expectedStiffness + externalK)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_massSpring->computeD(*m_initialState).isApprox(
										m_expectedDamping + m_expectedRayleighDamping + externalD)));
		EXPECT_NO_THROW(m_massSpring->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedForce + externalForce));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness + externalK));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + m_expectedRayleighDamping + externalD));
	}
}

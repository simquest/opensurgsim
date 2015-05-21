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

/// \file FemRepresentationTests.cpp
/// This file tests the non-abstract functionalities of the base class FemRepresentation

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/Framework/Runtime.h" ///< Used to initialize the Component Fem3DRepresentation
#include "SurgSim/Physics/UnitTests/MockObjects.h"

namespace SurgSim
{

namespace Physics
{

class FemRepresentationTests : public ::testing::Test
{
public:
	double m_rho;
	double m_nu;
	double m_E;
	double m_dt;
	double m_rayleighDampingMassParameter;
	double m_rayleighDampingStiffnessParameter;

	Vector m_expectedFemElementsForce;
	Vector m_expectedRayleighDampingForce;
	Vector m_expectedGravityForce;
	Matrix m_expectedMass;
	Matrix m_expectedDamping;
	Matrix m_expectedRayleighDamping;
	Matrix m_expectedStiffness;

	std::shared_ptr<MockFemRepresentation> m_fem;
	std::shared_ptr<SurgSim::Math::OdeState> m_initialState;

protected:
	void SetUp() override
	{
		m_rho = 2000.0;
		m_nu = 0.45;
		m_E = 1e6;
		m_dt = 1e-3;
		m_rayleighDampingMassParameter = 0.452;
		m_rayleighDampingStiffnessParameter = 0.242;

		m_fem = std::make_shared<MockFemRepresentation>("MockFem");

		m_initialState = std::make_shared<SurgSim::Math::OdeState>();
		m_initialState->setNumDof(m_fem->getNumDofPerNode(), 3);
		m_initialState->getVelocities().setOnes(); // v = (1...1) to test damping
		m_fem->setInitialState(m_initialState);

		std::shared_ptr<MockFemElement> element01 = std::make_shared<MockFemElement>();
		element01->setMassDensity(m_rho);
		element01->setPoissonRatio(m_nu);
		element01->setYoungModulus(m_E);
		element01->addNode(0);
		element01->addNode(1);
		m_fem->addFemElement(element01);

		std::shared_ptr<MockFemElement> element12 = std::make_shared<MockFemElement>();
		element12->setMassDensity(m_rho);
		element12->setPoissonRatio(m_nu);
		element12->setYoungModulus(m_E);
		element12->addNode(1);
		element12->addNode(2);
		m_fem->addFemElement(element12);

		// Mass should be a diagonal matrix with diagonal (1 1 1 2 2 2 1 1 1)
		m_expectedMass = Matrix::Zero(9, 9);
		m_expectedMass.diagonal() << 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 1.0, 1.0, 1.0;

		// Damping should be a diagonal matrix with diagonal (2 2 2 4 4 4 2 2 2)
		m_expectedDamping = Matrix::Zero(9, 9);
		m_expectedDamping.diagonal() << 2.0, 2.0, 2.0, 4.0, 4.0, 4.0, 2.0, 2.0, 2.0;

		// Stiffness should be a diagonal matrix with diagonal (3 3 3 6 6 6 3 3 3)
		m_expectedStiffness = Matrix::Zero(9, 9);
		m_expectedStiffness.diagonal() << 3.0, 3.0, 3.0, 6.0, 6.0, 6.0, 3.0, 3.0, 3.0;

		// Rayleigh damping = alpha.M + beta.K M and K are diagonals, so the resulting matrix is too
		m_expectedRayleighDamping = m_expectedMass * m_rayleighDampingMassParameter;
		m_expectedRayleighDamping += m_expectedStiffness * m_rayleighDampingStiffnessParameter;

		// FemElements force should be (1 2 3 4 5 6 0 0 0) + (0 0 0 1 2 3 4 5 6) = (1 2 3 5 7 9 4 5 6)
		m_expectedFemElementsForce.resize(9);
		m_expectedFemElementsForce << 1.0, 2.0, 3.0, 5.0, 7.0, 9.0, 4.0, 5.0, 6.0;

		// Gravity force should be m.gravity for each node of each element
		m_expectedGravityForce = Vector::Zero(9);
		SurgSim::Math::Vector3d g(0.0, -9.81, 0.0);
		m_expectedGravityForce.segment<3>(0) += g * element01->getMass(*m_initialState) / 2.0;
		m_expectedGravityForce.segment<3>(3) += g * element01->getMass(*m_initialState) / 2.0;
		m_expectedGravityForce.segment<3>(3) += g * element12->getMass(*m_initialState) / 2.0;
		m_expectedGravityForce.segment<3>(6) += g * element12->getMass(*m_initialState) / 2.0;

		// Rayleigh damping force should be -(alpha.M + beta.K).(1...1)^t
		// with (alpha.M + beta.K) a diagonal matrix
		m_expectedRayleighDampingForce = -m_expectedRayleighDamping.diagonal();
	}
};

TEST_F(FemRepresentationTests, ConstructorTest)
{
	ASSERT_NO_THROW({MockFemRepresentation fem("name");});
	ASSERT_NO_THROW({MockFemRepresentation* fem = new MockFemRepresentation("name"); delete fem;});
	ASSERT_NO_THROW({std::shared_ptr<MockFemRepresentation> fem = std::make_shared<MockFemRepresentation>("name");});
}

TEST_F(FemRepresentationTests, AddSetGetTest)
{
	MockFemRepresentation fem("name");

	/// Add/Get FemElement
	EXPECT_EQ(0u, fem.getNumFemElements());
	ASSERT_ANY_THROW(fem.getFemElement(0));
	ASSERT_ANY_THROW(fem.getFemElement(1));

	std::shared_ptr<MockFemElement> element = std::make_shared<MockFemElement>();
	element->setMassDensity(m_rho);
	element->setPoissonRatio(m_nu);
	element->setYoungModulus(m_E);
	ASSERT_NO_THROW(fem.addFemElement(element));
	EXPECT_EQ(1u, fem.getNumFemElements());
	EXPECT_NE(nullptr, fem.getFemElement(0));
	ASSERT_ANY_THROW(fem.getFemElement(1));

	element = std::make_shared<MockFemElement>();
	element->setMassDensity(m_rho);
	element->setPoissonRatio(m_nu);
	element->setYoungModulus(m_E);
	ASSERT_NO_THROW(fem.addFemElement(element));
	EXPECT_EQ(2u, fem.getNumFemElements());
	EXPECT_NE(nullptr, fem.getFemElement(0));
	EXPECT_NE(nullptr, fem.getFemElement(1));

	/// Test gets the total mass of the fem (Each MockFemElement has a fixed volume of 1m^3)
	EXPECT_DOUBLE_EQ(2.0 * m_rho, fem.getTotalMass());

	/// Test sets/gets the Rayleigh stiffness parameter
	EXPECT_DOUBLE_EQ(0.0, fem.getRayleighDampingStiffness());
	fem.setRayleighDampingStiffness(13.45);
	EXPECT_DOUBLE_EQ(13.45, fem.getRayleighDampingStiffness());

	/// Test sets/gets the Rayleigh mass parameter
	EXPECT_DOUBLE_EQ(0.0, fem.getRayleighDampingMass());
	fem.setRayleighDampingMass(43.99);
	EXPECT_DOUBLE_EQ(43.99, fem.getRayleighDampingMass());
}

TEST_F(FemRepresentationTests, IsValidCoordinateTest)
{
	using SurgSim::DataStructures::IndexedLocalCoordinate;

	MockFemRepresentation fem("name");

	std::shared_ptr<MockFemElement> element = std::make_shared<MockFemElement>();
	element->setMassDensity(m_rho);
	element->setPoissonRatio(m_nu);
	element->setYoungModulus(m_E);
	ASSERT_NO_THROW(fem.addFemElement(element));

	// Set up the fem element to have a valid natural coordinate.
	element->addNode(0);
	element->addNode(1);
	Vector validNaturalCoordinate(2);
	validNaturalCoordinate << 1.0, 0.0;

	// Only the elementId is tested (by passing a valid natural coordinate).
	// The validation of the natural coordinate is tested in the respective FemElement's UnitTest.
	IndexedLocalCoordinate invalidCoordinates(1, validNaturalCoordinate);
	EXPECT_FALSE(fem.isValidCoordinate(invalidCoordinates));
	IndexedLocalCoordinate validCoordinates(0, validNaturalCoordinate);
	EXPECT_TRUE(fem.isValidCoordinate(validCoordinates));
}

TEST_F(FemRepresentationTests, BeforeUpdateTest)
{
	MockFemRepresentation fem("name");

	// Throw exception (no FemElement)
	ASSERT_ANY_THROW(fem.beforeUpdate(m_dt));

	std::shared_ptr<MockFemElement> element = std::make_shared<MockFemElement>();
	element->setMassDensity(m_rho);
	element->setPoissonRatio(m_nu);
	element->setYoungModulus(m_E);
	fem.addFemElement(element);

	// Throw exception (no initialState setup yet)
	ASSERT_ANY_THROW(fem.beforeUpdate(m_dt));

	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(fem.getNumDofPerNode(), 8);
	fem.setInitialState(initialState);

	ASSERT_NO_THROW(fem.beforeUpdate(m_dt));
}

TEST_F(FemRepresentationTests, AfterUpdateTest)
{
	m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_fem->wakeUp();

	ASSERT_TRUE(m_fem->isActive());
	ASSERT_NO_THROW(m_fem->beforeUpdate(m_dt));
	ASSERT_TRUE(m_fem->isActive());
	ASSERT_NO_THROW(m_fem->update(m_dt));
	ASSERT_TRUE(m_fem->isActive());
	// After update should backup the currentState into finalState and update all FemElement
	ASSERT_NO_THROW(m_fem->afterUpdate(m_dt));
	ASSERT_FALSE(*m_fem->getFinalState() == *m_fem->getInitialState());
	ASSERT_TRUE(*m_fem->getFinalState() == *m_fem->getCurrentState());
	ASSERT_TRUE(m_fem->isActive());
}

TEST_F(FemRepresentationTests, ComputesWithNoGravityAndNoDampingTest)
{
	using SurgSim::Math::Vector3d;

	Vector* F;
	SparseMatrix* M, *D, *K;

	// No gravity, no Rayleigh damping
	// computeF tests addFemElementsForce
	m_fem->setIsGravityEnabled(false);
	m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_fem->wakeUp();

	Vector expectedF = m_expectedFemElementsForce;

	{
		SCOPED_TRACE("Without external force");

		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeF(*m_initialState).isApprox(expectedF)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeM(*m_initialState).isApprox(m_expectedMass)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeD(*m_initialState).isApprox(m_expectedDamping)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeK(*m_initialState).isApprox(m_expectedStiffness)));

		// Test combo method computeFMDK
		EXPECT_NO_THROW(m_fem->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedF));
		EXPECT_TRUE((*M).isApprox(m_expectedMass));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness));
	}

	{
		SCOPED_TRACE("With external force");

		std::shared_ptr<MockDeformableRepresentationLocalization> localization =
			std::make_shared<MockDeformableRepresentationLocalization>();
		localization->setRepresentation(m_fem);
		localization->setLocalNode(0);
		Vector FextLocal = Vector::Ones(m_fem->getNumDofPerNode());
		Matrix KextLocal = Matrix::Ones(m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode());
		Matrix DextLocal = KextLocal + Matrix::Identity(m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode());
		Vector Fext = Vector::Zero(m_fem->getNumDof());
		Fext.segment(0, m_fem->getNumDofPerNode()) = FextLocal;
		Matrix Kext = Matrix::Zero(m_fem->getNumDof(), m_fem->getNumDof());
		Kext.block(0, 0, m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode()) = KextLocal;
		Matrix Dext = Matrix::Zero(m_fem->getNumDof(), m_fem->getNumDof());
		Dext.block(0, 0, m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode()) = DextLocal;
		m_fem->addExternalGeneralizedForce(localization, FextLocal, KextLocal, DextLocal);

		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeF(*m_initialState).isApprox(expectedF + Fext)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeM(*m_initialState).isApprox(m_expectedMass)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeD(*m_initialState).isApprox(m_expectedDamping + Dext)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeK(*m_initialState).isApprox(m_expectedStiffness + Kext)));
		EXPECT_NO_THROW(m_fem->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedF + Fext));
		EXPECT_TRUE((*M).isApprox(m_expectedMass));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + Dext));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness + Kext));
	}
}

TEST_F(FemRepresentationTests, ComputesWithNoGravityAndDampingTest)
{
	using SurgSim::Math::Vector3d;

	Vector* F;
	SparseMatrix* M, *D, *K;

	// No gravity, Rayleigh damping
	// computeF tests addFemElementsForce and addRayleighDampingForce
	m_fem->setIsGravityEnabled(false);
	m_fem->setRayleighDampingMass(m_rayleighDampingMassParameter);
	m_fem->setRayleighDampingStiffness(m_rayleighDampingStiffnessParameter);
	m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_fem->wakeUp();

	SurgSim::Math::Vector expectedF = m_expectedFemElementsForce + m_expectedRayleighDampingForce;

	{
		SCOPED_TRACE("Without external force");

		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeF(*m_initialState).isApprox(expectedF)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeM(*m_initialState).isApprox(m_expectedMass)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeD(*m_initialState).isApprox(
										m_expectedDamping + m_expectedRayleighDamping)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeK(*m_initialState).isApprox(m_expectedStiffness)));
		// Test combo method computeFMDK
		EXPECT_NO_THROW(m_fem->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedF));
		EXPECT_TRUE((*M).isApprox(m_expectedMass));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + m_expectedRayleighDamping));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness));
	}

	{
		SCOPED_TRACE("With external force");

		std::shared_ptr<MockDeformableRepresentationLocalization> localization =
			std::make_shared<MockDeformableRepresentationLocalization>();
		localization->setRepresentation(m_fem);
		localization->setLocalNode(0);
		Vector FextLocal = Vector::Ones(m_fem->getNumDofPerNode());
		Matrix KextLocal = Matrix::Ones(m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode());
		Matrix DextLocal = KextLocal + Matrix::Identity(m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode());
		Vector Fext = Vector::Zero(m_fem->getNumDof());
		Fext.segment(0, m_fem->getNumDofPerNode()) = FextLocal;
		Matrix Kext = Matrix::Zero(m_fem->getNumDof(), m_fem->getNumDof());
		Kext.block(0, 0, m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode()) = KextLocal;
		Matrix Dext = Matrix::Zero(m_fem->getNumDof(), m_fem->getNumDof());
		Dext.block(0, 0, m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode()) = DextLocal;
		m_fem->addExternalGeneralizedForce(localization, FextLocal, KextLocal, DextLocal);

		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeF(*m_initialState).isApprox(expectedF + Fext)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeM(*m_initialState).isApprox(m_expectedMass)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeD(*m_initialState).isApprox(m_expectedDamping +
									m_expectedRayleighDamping + Dext)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeK(*m_initialState).isApprox(m_expectedStiffness + Kext)));
		EXPECT_NO_THROW(m_fem->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedF + Fext));
		EXPECT_TRUE((*M).isApprox(m_expectedMass));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + m_expectedRayleighDamping + Dext));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness + Kext));
	}
}

TEST_F(FemRepresentationTests, ComputesWithGravityAndNoDampingTest)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::SparseMatrix;

	Vector* F;
	SparseMatrix* M, *D, *K;

	// Gravity, no Rayleigh damping
	// computeF tests addFemElementsForce and addGravityForce
	m_fem->setIsGravityEnabled(true);
	m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_fem->wakeUp();

	SurgSim::Math::Vector expectedF = m_expectedFemElementsForce + m_expectedGravityForce;

	{
		SCOPED_TRACE("Without external force");

		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeF(*m_initialState).isApprox(expectedF)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeM(*m_initialState).isApprox(m_expectedMass)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeD(*m_initialState).isApprox(m_expectedDamping)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeK(*m_initialState).isApprox(m_expectedStiffness)));
		// Test combo method computeFMDK
		EXPECT_NO_THROW(m_fem->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedF));
		EXPECT_TRUE((*M).isApprox(m_expectedMass));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness));
	}

	{
		SCOPED_TRACE("With external force");

		std::shared_ptr<MockDeformableRepresentationLocalization> localization =
			std::make_shared<MockDeformableRepresentationLocalization>();
		localization->setRepresentation(m_fem);
		localization->setLocalNode(0);
		Vector FextLocal = Vector::Ones(m_fem->getNumDofPerNode());
		Matrix KextLocal = Matrix::Ones(m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode());
		Matrix DextLocal = KextLocal + Matrix::Identity(m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode());
		Vector Fext = Vector::Zero(m_fem->getNumDof());
		Fext.segment(0, m_fem->getNumDofPerNode()) = FextLocal;
		Matrix Kext = Matrix::Zero(m_fem->getNumDof(), m_fem->getNumDof());
		Kext.block(0, 0, m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode()) = KextLocal;
		Matrix Dext = Matrix::Zero(m_fem->getNumDof(), m_fem->getNumDof());
		Dext.block(0, 0, m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode()) = DextLocal;
		m_fem->addExternalGeneralizedForce(localization, FextLocal, KextLocal, DextLocal);

		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeF(*m_initialState).isApprox(expectedF + Fext)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeM(*m_initialState).isApprox(m_expectedMass)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeD(*m_initialState).isApprox(m_expectedDamping + Dext)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeK(*m_initialState).isApprox(m_expectedStiffness + Kext)));
		EXPECT_NO_THROW(m_fem->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedF + Fext));
		EXPECT_TRUE((*M).isApprox(m_expectedMass));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + Dext));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness + Kext));
	}
}

TEST_F(FemRepresentationTests, ComputesWithGravityAndDampingTest)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::SparseMatrix;

	Vector* F;
	SparseMatrix* M, *D, *K;

	// Gravity, Rayleigh damping
	// computeF tests addFemElementsForce, addRayleighDampingForce and addGravityForce
	m_fem->setIsGravityEnabled(true);
	m_fem->setRayleighDampingMass(m_rayleighDampingMassParameter);
	m_fem->setRayleighDampingStiffness(m_rayleighDampingStiffnessParameter);
	m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m_fem->wakeUp();

	SurgSim::Math::Vector expectedF = m_expectedFemElementsForce + m_expectedRayleighDampingForce +
									  m_expectedGravityForce;

	{
		SCOPED_TRACE("Without external force");

		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeF(*m_initialState).isApprox(expectedF)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeM(*m_initialState).isApprox(m_expectedMass)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeD(*m_initialState).isApprox(
										m_expectedDamping + m_expectedRayleighDamping)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeK(*m_initialState).isApprox(m_expectedStiffness)));
		// Test combo method computeFMDK
		EXPECT_NO_THROW(m_fem->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedF));
		EXPECT_TRUE((*M).isApprox(m_expectedMass));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + m_expectedRayleighDamping));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness));
	}

	{
		SCOPED_TRACE("With external force");

		std::shared_ptr<MockDeformableRepresentationLocalization> localization =
			std::make_shared<MockDeformableRepresentationLocalization>();
		localization->setRepresentation(m_fem);
		localization->setLocalNode(0);
		Vector FextLocal = Vector::Ones(m_fem->getNumDofPerNode());
		Matrix KextLocal = Matrix::Ones(m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode());
		Matrix DextLocal = KextLocal + Matrix::Identity(m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode());
		Vector Fext = Vector::Zero(m_fem->getNumDof());
		Fext.segment(0, m_fem->getNumDofPerNode()) = FextLocal;
		Matrix Kext = Matrix::Zero(m_fem->getNumDof(), m_fem->getNumDof());
		Kext.block(0, 0, m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode()) = KextLocal;
		Matrix Dext = Matrix::Zero(m_fem->getNumDof(), m_fem->getNumDof());
		Dext.block(0, 0, m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode()) = DextLocal;
		m_fem->addExternalGeneralizedForce(localization, FextLocal, KextLocal, DextLocal);

		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeF(*m_initialState).isApprox(expectedF + Fext)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeM(*m_initialState).isApprox(m_expectedMass)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeD(*m_initialState).isApprox(m_expectedDamping +
									m_expectedRayleighDamping + Dext)));
		EXPECT_NO_THROW(EXPECT_TRUE(m_fem->computeK(*m_initialState).isApprox(m_expectedStiffness + Kext)));
		EXPECT_NO_THROW(m_fem->computeFMDK(*m_initialState, &F, &M, &D, &K));
		EXPECT_TRUE((*F).isApprox(expectedF + Fext));
		EXPECT_TRUE((*M).isApprox(m_expectedMass));
		EXPECT_TRUE((*D).isApprox(m_expectedDamping + m_expectedRayleighDamping + Dext));
		EXPECT_TRUE((*K).isApprox(m_expectedStiffness + Kext));
	}
}

TEST_F(FemRepresentationTests, DoInitializeTest)
{
	using SurgSim::Framework::Runtime;

	MockFemRepresentation fem("name");

	// Setup the initial state
	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(fem.getNumDofPerNode(), 8);
	fem.setInitialState(initialState);

	// Add one element
	std::shared_ptr<MockFemElement> element = std::make_shared<MockFemElement>();
	element->setMassDensity(m_rho);
	element->setPoissonRatio(m_nu);
	element->setYoungModulus(m_E);
	element->addNode(0);
	element->addNode(1);
	element->addNode(2);
	fem.addFemElement(element);

	// Initialize the component, this will call FemRepresentation::doInitialize
	fem.initialize(std::make_shared<Runtime>());

	// At this point, all FemElement and m_massPerNode should have been initialized
	// We have 8 nodes, 1 element connecting nodes (0 1 2)
	// The mass should be evenly distributed on the 3 first nodes, and 0 for all others.
	std::shared_ptr<MockFemElement> femElement;
	ASSERT_NO_THROW(femElement = std::static_pointer_cast<MockFemElement>(fem.getFemElement(0)));
	ASSERT_TRUE(femElement->isInitialized());
	ASSERT_EQ(8u, fem.getMassPerNode().size());
	ASSERT_EQ(m_rho, fem.getTotalMass());
	for (size_t nodeId = 0; nodeId < 3; ++nodeId)
	{
		ASSERT_EQ(m_rho / 3.0, fem.getMassPerNode()[nodeId]);
	}
	for (size_t nodeId = 3; nodeId < 8; ++nodeId)
	{
		ASSERT_EQ(0u, fem.getMassPerNode()[nodeId]);
	}
}

TEST_F(FemRepresentationTests, ComplianceWarpingTest)
{
	{
		SCOPED_TRACE("MockFemRepresentation incomplete for compliance warping");
		auto fem = std::make_shared<MockFemRepresentation>("fem");

		EXPECT_NO_THROW(EXPECT_FALSE(fem->getComplianceWarping()));
		EXPECT_THROW(fem->setComplianceWarping(true), SurgSim::Framework::AssertionFailure);
		EXPECT_NO_THROW(EXPECT_FALSE(fem->getComplianceWarping()));

		// Setup the initial state
		auto initialState = std::make_shared<SurgSim::Math::OdeState>();
		initialState->setNumDof(fem->getNumDofPerNode(), 3);
		fem->setInitialState(initialState);

		// Add one element
		std::shared_ptr<MockFemElement> element = std::make_shared<MockFemElement>();
		element->setMassDensity(m_rho);
		element->setPoissonRatio(m_nu);
		element->setYoungModulus(m_E);
		element->addNode(0);
		element->addNode(1);
		element->addNode(2);
		fem->addFemElement(element);

		fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
		fem->wakeUp();

		EXPECT_NO_THROW(EXPECT_FALSE(fem->getComplianceWarping()));
		EXPECT_THROW(fem->setComplianceWarping(false), SurgSim::Framework::AssertionFailure);
		EXPECT_THROW(fem->setComplianceWarping(true), SurgSim::Framework::AssertionFailure);

		// update() will call updateNodesRotations() which will raise an exception in this case.
		// This method has not been overridden.
		// This has been disabled. No assertion will be thrown until compliance warping is back
		// in the code.
		EXPECT_NO_THROW(fem->update(1e-3));
	}

	{
		SCOPED_TRACE("MockFemRepresentation complete for compliance warping");
		auto fem = std::make_shared<MockFemRepresentationValidComplianceWarping>("fem");

		EXPECT_NO_THROW(EXPECT_FALSE(fem->getComplianceWarping()));
		EXPECT_THROW(fem->setComplianceWarping(true), SurgSim::Framework::AssertionFailure);
		EXPECT_NO_THROW(EXPECT_FALSE(fem->getComplianceWarping()));

		// Setup the initial state
		auto initialState = std::make_shared<SurgSim::Math::OdeState>();
		initialState->setNumDof(fem->getNumDofPerNode(), 3);
		fem->setInitialState(initialState);

		// Add one element
		std::shared_ptr<MockFemElement> element = std::make_shared<MockFemElement>();
		element->setMassDensity(m_rho);
		element->setPoissonRatio(m_nu);
		element->setYoungModulus(m_E);
		element->addNode(0);
		element->addNode(1);
		element->addNode(2);
		fem->addFemElement(element);

		fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
		fem->wakeUp();

		EXPECT_NO_THROW(EXPECT_FALSE(fem->getComplianceWarping()));
		EXPECT_THROW(fem->setComplianceWarping(false), SurgSim::Framework::AssertionFailure);
		EXPECT_THROW(fem->setComplianceWarping(true), SurgSim::Framework::AssertionFailure);

		// update() will call updateNodesRotations() which will not raise an exception in this case.
		// This method has been overridden.
		EXPECT_NO_THROW(fem->update(1e-3));

		EXPECT_NO_THROW(EXPECT_TRUE((fem->applyCompliance(*initialState, Matrix::Identity(initialState->getNumDof(),
									 initialState->getNumDof())) / 1e-3).isIdentity()));
	}
}

TEST_F(FemRepresentationTests, SerializationTest)
{
	auto fem = std::make_shared<MockFemRepresentation>("Test-Fem");

	EXPECT_THROW(fem->setValue("ComplianceWarping", true), SurgSim::Framework::AssertionFailure);
	EXPECT_FALSE(fem->getComplianceWarping());
	EXPECT_FALSE(fem->getValue<bool>("ComplianceWarping"));
	EXPECT_NO_THROW(fem->setValue("ComplianceWarping", false));
	EXPECT_FALSE(fem->getComplianceWarping());
	EXPECT_FALSE(fem->getValue<bool>("ComplianceWarping"));
}

} // namespace Physics

} // namespace SurgSim

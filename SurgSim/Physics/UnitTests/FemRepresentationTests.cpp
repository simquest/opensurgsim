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

#include "SurgSim/Framework/Runtime.h" //< Used to initialize the Component Fem3DRepresentation

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

protected:
	virtual void SetUp() override
	{
		m_rho = 2000.0;
		m_nu = 0.45;
		m_E = 1e6;
		m_dt = 1e-3;
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

TEST_F(FemRepresentationTests, BeforeUpdateTest)
{
	MockFemRepresentation fem("name");

	// Throw exception (no FemElement + no initialState setup yet)
	EXPECT_EQ(nullptr, fem.getOdeSolver());
	ASSERT_ANY_THROW(fem.beforeUpdate(m_dt));
	EXPECT_EQ(nullptr, fem.getOdeSolver());
	std::shared_ptr<MockFemElement> element = std::make_shared<MockFemElement>();
	element->setMassDensity(m_rho);
	element->setPoissonRatio(m_nu);
	element->setYoungModulus(m_E);
	fem.addFemElement(element);

	// Throw exception (no initialState setup yet)
	ASSERT_ANY_THROW(fem.beforeUpdate(m_dt));
	EXPECT_EQ(nullptr, fem.getOdeSolver());

	// No exception (FemElement and initial state setup)
	std::shared_ptr<DeformableRepresentationState> initialState =
		std::make_shared<DeformableRepresentationState>();
	initialState->setNumDof(fem.getNumDofPerNode(), 8);
	fem.setInitialState(initialState);
	ASSERT_NO_THROW(fem.beforeUpdate(m_dt));
	// The OdeSolver should have been initialized properly
	EXPECT_NE(nullptr, fem.getOdeSolver());
}

TEST_F(FemRepresentationTests, UpdateTest)
{
	// Nothing to test, we need an actual FemElement doing something.
	// Concrete derived classes will do this test
}

TEST_F(FemRepresentationTests, AfterUpdateTest)
{
	// Nothing to test, we need an actual FemElement doing something.
	// Concrete derived classes will do this test
}

TEST_F(FemRepresentationTests, ApplyCorrectionTest)
{
	// Nothing to test, we need an actual FemElement doing something.
	// Concrete derived classes will do this test
}

TEST_F(FemRepresentationTests, ComputesTest)
{
	// Nothing to test, we need an actual FemElement doing something.
	// Concrete derived classes will do this test
}

TEST_F(FemRepresentationTests, InitializeTest)
{
	using SurgSim::Framework::Runtime;

	MockFemRepresentation fem("name");
	std::shared_ptr<DeformableRepresentationState> initialState =
		std::make_shared<DeformableRepresentationState>();
	initialState->setNumDof(fem.getNumDofPerNode(), 8);

	std::shared_ptr<SurgSim::Physics::DeformableRepresentationState> state =
		std::make_shared<SurgSim::Physics::DeformableRepresentationState>();
	state->setNumDof(fem.getNumDofPerNode(), 4);

	// Initial state not setup yet for the fem + Initialize not called
	ASSERT_ANY_THROW({SurgSim::Math::Vector F = fem.computeF(*state);});

	std::shared_ptr<MockFemElement> element = std::make_shared<MockFemElement>();
	element->setMassDensity(m_rho);
	element->setPoissonRatio(m_nu);
	element->setYoungModulus(m_E);
	fem.addFemElement(element);
	ASSERT_ANY_THROW({SurgSim::Math::Vector F = fem.computeF(*state);});

	fem.setInitialState(initialState);
	// Initial state setup (and we even have 1 FemElement) BUT Initialize not called yet
	// Note as well that the number of nodes don't match between initialState and state
	ASSERT_ANY_THROW({SurgSim::Math::Vector F = fem.computeF(*state);});

	fem.initialize(std::make_shared<Runtime>());
	// Initial state setup (and we even have 1 FemElement) + Initialize called
	// BUTE note as well that the number of nodes don't match between initialState and state
	ASSERT_ANY_THROW({SurgSim::Math::Vector F = fem.computeF(*state);});

	// Initial state setup (and we even have 1 FemElement) + Initialize called
	ASSERT_NO_THROW({SurgSim::Math::Vector F = fem.computeF(*initialState);});
}

} // namespace Physics

} // namespace SurgSim

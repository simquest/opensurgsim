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

#include <memory>
#include <string>

#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemElement3DTetrahedron.h"
#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"
using SurgSim::Physics::Fem3DRepresentation;
using SurgSim::Physics::Fem3DRepresentationLocalization;

#include "SurgSim/Math/Vector.h"

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Physics
{

void addTetraheadron(Fem3DRepresentation *fem, std::array<unsigned int, 4> nodes,
					 const DeformableRepresentationState &state, double massDensity = 1.0,
					 double poissonRatio = 0.1, double youngModulus = 1.0)
{
	auto element = std::make_shared<FemElement3DTetrahedron>(nodes, state);
	element->setMassDensity(massDensity);
	element->setPoissonRatio(poissonRatio);
	element->setYoungModulus(youngModulus);
	element->initialize(state);
	fem->addFemElement(element);
}

class Fem3DRepresentationLocalizationTest : public ::testing::Test
{
public:
	void SetUp()
	{
		using SurgSim::Math::Vector3d;

		m_fem = std::make_shared<Fem3DRepresentation>("Fem3dRepresentation");
		auto state = std::make_shared<DeformableRepresentationState>();
		state->setNumDof(3, 6);

		state->getPositions().segment<3>(0 * 3) = Vector3d( 0.0,  0.0,  0.0);
		state->getPositions().segment<3>(1 * 3) = Vector3d( 0.0,  1.0, -1.0);
		state->getPositions().segment<3>(2 * 3) = Vector3d(-1.0,  1.0,  0.0);
		state->getPositions().segment<3>(3 * 3) = Vector3d( 0.0,  1.0,  0.0);
		state->getPositions().segment<3>(4 * 3) = Vector3d( 1.0,  1.0,  0.0);
		state->getPositions().segment<3>(5 * 3) = Vector3d( 1.0,  0.0, -1.0);

		{
			std::array<unsigned int, 4> nodes = {0, 1, 2, 3};
			addTetraheadron(m_fem.get(), nodes, *state);
		}

		{
			std::array<unsigned int, 4> nodes = {0, 1, 3, 4};
			addTetraheadron(m_fem.get(), nodes, *state);
		}

		{
			std::array<unsigned int, 4> nodes = {0, 1, 4, 5};
			addTetraheadron(m_fem.get(), nodes, *state);
		}

		m_fem->setInitialState(state);
		m_fem->setIsActive(true);
	}

	void TearDown()
	{
	}

	std::shared_ptr<Fem3DRepresentation> m_fem;
};

TEST_F(Fem3DRepresentationLocalizationTest, ConstructorTest)
{
	ASSERT_NO_THROW({
		Fem3DRepresentationLocalization localization;
	});

	ASSERT_NO_THROW({
		Fem3DRepresentationLocalization localization(m_fem);
	});
}

TEST_F(Fem3DRepresentationLocalizationTest, SetGetRepresentation)
{
	Fem3DRepresentationLocalization localization;

	EXPECT_EQ(nullptr, localization.getRepresentation());

	localization.setRepresentation(m_fem);
	EXPECT_EQ(m_fem, localization.getRepresentation());

	localization.setRepresentation(nullptr);
	EXPECT_EQ(nullptr, localization.getRepresentation());
}

TEST_F(Fem3DRepresentationLocalizationTest, FemRepresentationCoordinate)
{
	using SurgSim::Math::Vector4d;

	ASSERT_NO_THROW({
		FemRepresentationCoordinate coord;
	});

	ASSERT_NO_THROW({
		FemRepresentationCoordinate coord(6u, Vector4d(0.25, 0.55, 0.73, 0.11));
	});

	{
		FemRepresentationCoordinate coord(6u, Vector4d(0.25, 0.55, 0.73, 0.11));
		EXPECT_EQ(6u, coord.elementId);
		EXPECT_TRUE(Vector4d(0.25, 0.55, 0.73, 0.11).isApprox(coord.naturalCoordinate));
	}

	{
		FemRepresentationCoordinate coord;
		coord.elementId = 12u;
		coord.naturalCoordinate = Vector4d(0.33, 0.1, 0.05, 0.99);
		EXPECT_EQ(12u, coord.elementId);
		EXPECT_TRUE(Vector4d(0.33, 0.1, 0.05, 0.99).isApprox(coord.naturalCoordinate));
	}
}

TEST_F(Fem3DRepresentationLocalizationTest, SetGetLocalization)
{
	using SurgSim::Math::Vector4d;
	using SurgSim::Math::Vector3d;

	{
		// Uninitialized Representation
		auto localization = std::make_shared<Fem3DRepresentationLocalization>();
		EXPECT_THROW(localization->setLocalPosition(FemRepresentationCoordinate(0u, Vector4d(1.0, 0.0, 0.0, 0.0))),
			SurgSim::Framework::AssertionFailure);
	}

	{
		// Incorrectly formed natural coordinate
		auto localization = std::make_shared<Fem3DRepresentationLocalization>(m_fem);
		EXPECT_THROW(localization->setLocalPosition(FemRepresentationCoordinate(0u, Vector4d(0.25, 0.55, 0.73, 0.11))),
			SurgSim::Framework::AssertionFailure);

		EXPECT_THROW(localization->setLocalPosition(FemRepresentationCoordinate(0u, Vector3d(1.0, 0.0, 0.0))),
			SurgSim::Framework::AssertionFailure);
	}

	{
		// Out of bounds element Id
		auto localization = std::make_shared<Fem3DRepresentationLocalization>(m_fem);
		EXPECT_THROW(localization->setLocalPosition(FemRepresentationCoordinate(6u, Vector4d(1.0, 0.0, 0.0, 0.0))),
			SurgSim::Framework::AssertionFailure);
	}

	{
		auto localization = std::make_shared<Fem3DRepresentationLocalization>(m_fem);
		EXPECT_NO_THROW(localization->setLocalPosition(FemRepresentationCoordinate(1u, Vector4d(0.1, 0.1, 0.4, 0.4))));
		EXPECT_EQ(1u, localization->getLocalPosition().elementId);
		EXPECT_TRUE(Vector4d(0.1, 0.1, 0.4, 0.4).isApprox(localization->getLocalPosition().naturalCoordinate));
	}
}

TEST_F(Fem3DRepresentationLocalizationTest, CalculatePositionTest)
{
	using SurgSim::Math::Vector;
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Vector4d;

	auto localization = std::make_shared<Fem3DRepresentationLocalization>(m_fem);

	// Test tetrahedron 1: nodes 0, 1, 2, 3
	localization->setLocalPosition(FemRepresentationCoordinate(0u, Vector4d(1.0, 0.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 0.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(FemRepresentationCoordinate(0u, Vector4d(0.0, 1.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(FemRepresentationCoordinate(0u, Vector4d(0.0, 0.0, 1.0, 0.0)));
	EXPECT_TRUE(Vector3d(-1.0, 1.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(FemRepresentationCoordinate(0u, Vector4d(0.0, 0.0, 0.0, 1.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	// Test tetrahedron 2: nodes 0, 1, 3, 4
	localization->setLocalPosition(FemRepresentationCoordinate(1u, Vector4d(1.0, 0.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 0.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(FemRepresentationCoordinate(1u, Vector4d(0.0, 1.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(FemRepresentationCoordinate(1u, Vector4d(0.0, 0.0, 1.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(FemRepresentationCoordinate(1u, Vector4d(0.0, 0.0, 0.0, 1.0)));
	EXPECT_TRUE(Vector3d(1.0, 1.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	// Test tetrahedron 3: nodes 0, 1, 4, 5
	localization->setLocalPosition(FemRepresentationCoordinate(2u, Vector4d(1.0, 0.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 0.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(FemRepresentationCoordinate(2u, Vector4d(0.0, 1.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(FemRepresentationCoordinate(2u, Vector4d(0.0, 0.0, 1.0, 0.0)));
	EXPECT_TRUE(Vector3d(1.0, 1.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(FemRepresentationCoordinate(2u, Vector4d(0.0, 0.0, 0.0, 1.0)));
	EXPECT_TRUE(Vector3d(1.0, 0.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	// Advanced tests
	localization->setLocalPosition(FemRepresentationCoordinate(0u, Vector4d(0.31, 0.03, 0.19, 0.47)));
	//   0.31 * ( 0.0,  0.0,  0.0) => ( 0.0,  0.0,   0.0 )
	// + 0.03 * ( 0.0,  1.0, -1.0) => ( 0.0,  0.03, -0.03)
	// + 0.19 * (-1.0,  1.0,  0.0) => (-0.19, 0.19,  0.0 )
	// + 0.47 * ( 0.0,  1.0,  0.0) => ( 0.0,  0.47,  0.0 )
	//                              = (-0.19, 0.69, -0.03)
	EXPECT_TRUE(Vector3d(-0.19, 0.69, -0.03).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(FemRepresentationCoordinate(1u, Vector4d(0.05, 0.81, 0.06, 0.08)));
	//   0.05 * ( 0.0,  0.0,  0.0) => (0.0,  0.0,   0.0 )
	// + 0.81 * ( 0.0,  1.0, -1.0) => (0.0,  0.81, -0.81)
	// + 0.06 * ( 0.0,  1.0,  0.0) => (0.0,  0.06,  0.0 )
	// + 0.08 * ( 1.0,  1.0,  0.0) => (0.08, 0.08,  0.0 )
	//                              = (0.08, 0.95, -0.81)
	EXPECT_TRUE(Vector3d(0.08, 0.95, -0.81).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(FemRepresentationCoordinate(2u, Vector4d(0.11, 0.15, 0.67, 0.07)));
	//   0.11 * ( 0.0,  0.0,  0.0) => (0.0,  0.0,   0.0 )
	// + 0.15 * ( 0.0,  1.0, -1.0) => (0.0,  0.15, -0.15)
	// + 0.67 * ( 1.0,  1.0,  0.0) => (0.67, 0.67,  0.0 )
	// + 0.07 * ( 1.0,  0.0, -1.0) => (0.07, 0.0,  -0.07)
	//                              = (0.74, 0.82, -0.22)
	EXPECT_TRUE(Vector3d(0.74, 0.82, -0.22).isApprox(localization->calculatePosition(), epsilon));
}

} // namespace SurgSim
} // namespace Physics

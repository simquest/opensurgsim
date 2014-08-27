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

#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem1DRepresentationLocalization.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"

using SurgSim::DataStructures::IndexedLocalCoordinate;

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Physics
{

void addBeam(Fem1DRepresentation *fem, std::array<size_t, 2> nodes,
	const SurgSim::Math::OdeState& state, double radius = 0.01,
	double massDensity = 1.0, double poissonRatio = 0.1, double youngModulus = 1.0)
{
	auto element = std::make_shared<Fem1DElementBeam>(nodes);
	element->setRadius(radius);
	element->setMassDensity(massDensity);
	element->setPoissonRatio(poissonRatio);
	element->setYoungModulus(youngModulus);
	element->initialize(state);
	fem->addFemElement(element);
}

class Fem1DRepresentationLocalizationTest : public ::testing::Test
{
public:
	void SetUp()
	{
		using SurgSim::Math::Vector3d;
		using SurgSim::Math::getSubVector;

		m_fem = std::make_shared<Fem1DRepresentation>("Fem1dRepresentation");
		auto state = std::make_shared<SurgSim::Math::OdeState>();
		state->setNumDof(6, 3);

		auto& x = state->getPositions();
		getSubVector(x, 0, 6).segment<3>(0) = Vector3d( 0.0,  0.0,  0.0);
		getSubVector(x, 1, 6).segment<3>(0) = Vector3d( 0.0,  1.0, -1.0);
		getSubVector(x, 2, 6).segment<3>(0) = Vector3d(-1.0,  1.0,  0.0);

		// Define Beams
		{
			std::array<size_t, 2> nodes = {{0, 1}};
			addBeam(m_fem.get(), nodes, *state);
		}

		{
			std::array<size_t, 2> nodes = {{1, 2}};
			addBeam(m_fem.get(), nodes, *state);
		}

		m_fem->setInitialState(state);
		m_fem->setIsActive(true);
	}

	void TearDown()
	{
	}

	std::shared_ptr<Fem1DRepresentation> m_fem;
};

TEST_F(Fem1DRepresentationLocalizationTest, ConstructorTest)
{
	ASSERT_NO_THROW({
		Fem1DRepresentationLocalization localization;
	});

	ASSERT_NO_THROW({
		Fem1DRepresentationLocalization localization(m_fem);
	});
}

TEST_F(Fem1DRepresentationLocalizationTest, SetGetRepresentation)
{
	Fem1DRepresentationLocalization localization;

	EXPECT_EQ(nullptr, localization.getRepresentation());

	localization.setRepresentation(m_fem);
	EXPECT_EQ(m_fem, localization.getRepresentation());

	localization.setRepresentation(nullptr);
	EXPECT_EQ(nullptr, localization.getRepresentation());
}

TEST_F(Fem1DRepresentationLocalizationTest, SetGetLocalization)
{
	using SurgSim::Math::Vector2d;
	using SurgSim::Math::Vector3d;

	{
		// Uninitialized Representation
		auto localization = std::make_shared<Fem1DRepresentationLocalization>();
		EXPECT_THROW(localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector2d(1.0, 0.0))),
			SurgSim::Framework::AssertionFailure);
	}

	{
		// Incorrectly formed natural coordinate
		auto localization = std::make_shared<Fem1DRepresentationLocalization>(m_fem);
		EXPECT_THROW(localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector2d(0.89, 0.54))),
			SurgSim::Framework::AssertionFailure);

		EXPECT_THROW(localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector3d(1.0, 0.0, 0.0))),
			SurgSim::Framework::AssertionFailure);
	}

	{
		// Out of bounds element Id
		auto localization = std::make_shared<Fem1DRepresentationLocalization>(m_fem);
		EXPECT_THROW(localization->setLocalPosition(IndexedLocalCoordinate(6u, Vector2d(1.0, 0.0))),
			SurgSim::Framework::AssertionFailure);
	}

	{
		auto localization = std::make_shared<Fem1DRepresentationLocalization>(m_fem);
		EXPECT_NO_THROW(localization->setLocalPosition(IndexedLocalCoordinate(1u, Vector2d(0.2, 0.8))));
		EXPECT_EQ(1u, localization->getLocalPosition().index);
		EXPECT_TRUE(Vector2d(0.2, 0.8).isApprox(localization->getLocalPosition().coordinate));
	}
}

TEST_F(Fem1DRepresentationLocalizationTest, CalculatePositionTest)
{
	using SurgSim::Math::Vector;
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Vector2d;

	auto localization = std::make_shared<Fem1DRepresentationLocalization>(m_fem);

	// Test beam 1: nodes 0, 1
	localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector2d(1.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 0.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector2d(0.0, 1.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	// Test beam 2: nodes 0, 1
	localization->setLocalPosition(IndexedLocalCoordinate(1u, Vector2d(1.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(1u, Vector2d(0.0, 1.0)));
	EXPECT_TRUE(Vector3d(-1.0, 1.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	// Advanced tests
	localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector2d(0.31, 0.69)));
	//   0.31 * ( 0.0, 0.0, 0.0) => ( 0.0, 0.0,  0.0 )
	// + 0.69 * ( 0.0, 1.0,-1.0) => ( 0.0, 0.69,-0.69)
	//                            = ( 0.0, 0.69,-0.69)
	EXPECT_TRUE(Vector3d(0.0, 0.69, -0.69).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(1u, Vector2d(0.95, 0.05)));
	//   0.95 * ( 0.0, 1.0,-1.0) => ( 0.0,  0.95,-0.95)
	// + 0.05 * (-1.0, 1.0, 0.0) => (-0.05, 0.05, 0.0 )
	//                            = (-0.05, 1.0, -0.95)
	EXPECT_TRUE(Vector3d(-0.05, 1.0, -0.95).isApprox(localization->calculatePosition(), epsilon));
}

} // namespace SurgSim
} // namespace Physics

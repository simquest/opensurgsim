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
#include "SurgSim/Physics/UnitTests/MockObjects.h"
#include "SurgSim/Physics/FemLocalization.h"
#include "SurgSim/Physics/FemRepresentation.h"

using SurgSim::DataStructures::IndexedLocalCoordinate;

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Physics
{

std::shared_ptr<MockFemElement> createFemElement(std::array<size_t, 2> nodes)
{
	auto element = std::make_shared<MockFemElement>();
	for (const auto& node : nodes) element->addNode(node);
	element->setMassDensity(1000.0);
	element->setPoissonRatio(0.45);
	element->setYoungModulus(1e6);
	return element;
}

class FemLocalizationTest : public ::testing::Test
{
public:
	void SetUp()
	{
		using SurgSim::Math::Vector3d;
		using SurgSim::Math::getSubVector;

		m_fem = std::make_shared<MockFemRepresentation>("MockFemRepresentation");
		auto state = std::make_shared<SurgSim::Math::OdeState>();
		state->setNumDof(6, 3);

		auto& x = state->getPositions();
		getSubVector(x, 0, 6).segment<3>(0) = Vector3d( 0.0,  0.0,  0.0);
		getSubVector(x, 1, 6).segment<3>(0) = Vector3d( 0.0,  1.0, -1.0);
		getSubVector(x, 2, 6).segment<3>(0) = Vector3d(-1.0,  1.0,  0.0);

		// Define Beams
		{
			std::array<size_t, 2> nodes = {{0, 1}};
			m_fem->addFemElement(createFemElement(nodes));
		}

		{
			std::array<size_t, 2> nodes = {{1, 2}};
			m_fem->addFemElement(createFemElement(nodes));
		}

		m_fem->setInitialState(state);
		m_fem->setLocalActive(true);

		m_validLocalPosition.index = 1;
		m_validLocalPosition.coordinate = SurgSim::Math::Vector::Zero(2);
		m_validLocalPosition.coordinate[0] = 0.4;
		m_validLocalPosition.coordinate[1] = 0.6;

		m_invalidIndexLocalPosition.index = 3;
		m_invalidIndexLocalPosition.coordinate = SurgSim::Math::Vector::Zero(2);
		m_invalidIndexLocalPosition.coordinate[0] = 0.4;
		m_invalidIndexLocalPosition.coordinate[1] = 0.6;

		m_invalidCoordinateLocalPosition.index = 1;
		m_invalidCoordinateLocalPosition.coordinate = SurgSim::Math::Vector::Zero(2);
		m_invalidCoordinateLocalPosition.coordinate[0] = 0.6;
		m_invalidCoordinateLocalPosition.coordinate[1] = 0.6;
	}

	void TearDown()
	{
	}

	std::shared_ptr<MockFemRepresentation> m_fem;
	SurgSim::DataStructures::IndexedLocalCoordinate m_validLocalPosition;
	SurgSim::DataStructures::IndexedLocalCoordinate m_invalidIndexLocalPosition;
	SurgSim::DataStructures::IndexedLocalCoordinate m_invalidCoordinateLocalPosition;
};

TEST_F(FemLocalizationTest, ConstructorTest)
{
	// IndexedLocalCoordinate pointing to a node (node index + empty coordinate) are invalid. It will failed,
	// either because the index is out of bound or because the coordinates are the wrong size (empty)
	// This is tested by m_invalidIndexLocalPosition and m_invalidCoordinateLocalPosition
	ASSERT_THROW(
		std::make_shared<FemLocalization>(m_fem, m_invalidIndexLocalPosition),
		SurgSim::Framework::AssertionFailure);

	ASSERT_THROW(
		std::make_shared<FemLocalization>(m_fem, m_invalidCoordinateLocalPosition),
		SurgSim::Framework::AssertionFailure);

	ASSERT_NO_THROW(std::make_shared<FemLocalization>(m_fem, m_validLocalPosition););
}

TEST_F(FemLocalizationTest, SetGetRepresentation)
{
	FemLocalization localization(m_fem, m_validLocalPosition);

	EXPECT_NE(nullptr, localization.getRepresentation());
	EXPECT_EQ(m_fem, localization.getRepresentation());

	EXPECT_EQ(1u, localization.getLocalPosition().index);
	EXPECT_TRUE(localization.getLocalPosition().coordinate.isApprox(m_validLocalPosition.coordinate));

	localization.setRepresentation(nullptr);
	EXPECT_EQ(nullptr, localization.getRepresentation());
	localization.setRepresentation(m_fem);
	EXPECT_EQ(m_fem, localization.getRepresentation());

	SurgSim::DataStructures::IndexedLocalCoordinate m_otherValidLocalPosition;
	m_otherValidLocalPosition.index = 0;
	m_otherValidLocalPosition.coordinate = SurgSim::Math::Vector::Zero(2);
	m_otherValidLocalPosition.coordinate[1] = 1.0;

	localization.setLocalPosition(m_otherValidLocalPosition);
	EXPECT_EQ(m_otherValidLocalPosition.index, localization.getLocalPosition().index);
	EXPECT_TRUE(localization.getLocalPosition().coordinate.isApprox(m_otherValidLocalPosition.coordinate));
}

TEST_F(FemLocalizationTest, SetGetLocalization)
{
	using SurgSim::Math::Vector2d;
	using SurgSim::Math::Vector3d;

	{
		SCOPED_TRACE("Uninitialized Representation");

		// Uninitialized Representation
		EXPECT_THROW(std::make_shared<FemLocalization>(nullptr, m_validLocalPosition),
			SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Incorrectly formed natural coordinate");

		// Incorrectly formed natural coordinate
		auto localization = std::make_shared<FemLocalization>(m_fem, m_validLocalPosition);
		EXPECT_THROW(localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector2d(0.89, 0.54))),
			SurgSim::Framework::AssertionFailure);

		EXPECT_THROW(localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector3d(1.0, 0.0, 0.0))),
			SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Out of bounds element Id");

		// Out of bounds element Id
		auto localization = std::make_shared<FemLocalization>(m_fem, m_validLocalPosition);
		EXPECT_THROW(localization->setLocalPosition(IndexedLocalCoordinate(6u, Vector2d(1.0, 0.0))),
			SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("valid");

		auto localization = std::make_shared<FemLocalization>(m_fem, m_validLocalPosition);
		EXPECT_NO_THROW(localization->setLocalPosition(IndexedLocalCoordinate(1u, Vector2d(0.2, 0.8))));
		EXPECT_EQ(1u, localization->getLocalPosition().index);
		EXPECT_TRUE(Vector2d(0.2, 0.8).isApprox(localization->getLocalPosition().coordinate));
	}
}

TEST_F(FemLocalizationTest, CalculatePositionTest)
{
	using SurgSim::Math::Vector;
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Vector2d;

	auto localization = std::make_shared<FemLocalization>(m_fem, m_validLocalPosition);

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

TEST_F(FemLocalizationTest, ElementPose)
{
	FemLocalization localization(m_fem, m_validLocalPosition);
	EXPECT_THROW(localization.getElementPose(), SurgSim::Framework::AssertionFailure);
}

TEST_F(FemLocalizationTest, MoveClosestTo)
{
	FemLocalization localization(m_fem, m_validLocalPosition);
	bool flag = false;
	EXPECT_THROW(localization.moveClosestTo(SurgSim::Math::Vector3d::Zero(), &flag),
		SurgSim::Framework::AssertionFailure);
}

} // namespace SurgSim
} // namespace Physics

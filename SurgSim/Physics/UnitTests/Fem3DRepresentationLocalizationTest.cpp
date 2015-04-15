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
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"
#include "SurgSim/Physics/Fem3DElementCube.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"

using SurgSim::DataStructures::IndexedLocalCoordinate;
using SurgSim::Math::getSubVector;

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Physics
{

void addTetraheadron(Fem3DRepresentation *fem, std::array<size_t, 4> nodes,
	const SurgSim::Math::OdeState& state, double massDensity = 1.0,
	double poissonRatio = 0.1, double youngModulus = 1.0)
{
	auto element = std::make_shared<Fem3DElementTetrahedron>(nodes);
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
		auto state = std::make_shared<SurgSim::Math::OdeState>();
		state->setNumDof(3, 6);

		auto& x = state->getPositions();
		getSubVector(x, 0, 3) = Vector3d( 0.0,  0.0,  0.0);
		getSubVector(x, 1, 3) = Vector3d( 0.0,  1.0, -1.0);
		getSubVector(x, 2, 3) = Vector3d(-1.0,  1.0,  0.0);
		getSubVector(x, 3, 3) = Vector3d( 0.0,  1.0,  0.0);
		getSubVector(x, 4, 3) = Vector3d( 1.0,  1.0,  0.0);
		getSubVector(x, 5, 3) = Vector3d( 1.0,  0.0, -1.0);

		// Define Tetrahedrons
		{
			std::array<size_t, 4> nodes = {{0, 1, 2, 3}};
			addTetraheadron(m_fem.get(), nodes, *state);
		}

		{
			std::array<size_t, 4> nodes = {{0, 1, 3, 4}};
			addTetraheadron(m_fem.get(), nodes, *state);
		}

		{
			std::array<size_t, 4> nodes = {{0, 1, 4, 5}};
			addTetraheadron(m_fem.get(), nodes, *state);
		}

		m_fem->setInitialState(state);
		m_fem->setLocalActive(true);

		// FEMRepresentation for Fem3DElementCube
		m_fem3DCube = std::make_shared<Fem3DRepresentation>("Fem3dCubeRepresentation");
		auto restState = std::make_shared<SurgSim::Math::OdeState>();
		restState->setNumDof(3, 8);

		auto& x0 = restState->getPositions();
		getSubVector(x0, 0, 3) = Vector3d(-1.0,-1.0,-1.0);
		getSubVector(x0, 1, 3) = Vector3d( 1.0,-1.0,-1.0);
		getSubVector(x0, 2, 3) = Vector3d(-1.0, 1.0,-1.0);
		getSubVector(x0, 3, 3) = Vector3d( 1.0, 1.0,-1.0);
		getSubVector(x0, 4, 3) = Vector3d(-1.0,-1.0, 1.0);
		getSubVector(x0, 5, 3) = Vector3d( 1.0,-1.0, 1.0);
		getSubVector(x0, 6, 3) = Vector3d(-1.0, 1.0, 1.0);
		getSubVector(x0, 7, 3) = Vector3d( 1.0, 1.0, 1.0);

		// Define Cube
		{
			std::array<size_t, 8> node0 = {{0, 1, 3, 2, 4, 5, 7, 6}};
			std::shared_ptr<Fem3DElementCube> femElement = std::make_shared<Fem3DElementCube>(node0);
			femElement->setMassDensity(1.0);
			femElement->setPoissonRatio(0.1);
			femElement->setYoungModulus(1.0);

			m_fem3DCube->addFemElement(femElement);
		}
		m_fem3DCube->setInitialState(restState);
		m_fem3DCube->setLocalActive(true);

		m_validLocalPosition.index = 1;
		m_validLocalPosition.coordinate = SurgSim::Math::Vector::Zero(4);
		m_validLocalPosition.coordinate[0] = 0.4;
		m_validLocalPosition.coordinate[1] = 0.6;

		m_validLocalPositionForCube.index = 0;
		m_validLocalPositionForCube.coordinate = SurgSim::Math::Vector::Zero(8);
		m_validLocalPositionForCube.coordinate[0] = 0.4;
		m_validLocalPositionForCube.coordinate[1] = 0.6;

		m_invalidIndexLocalPosition.index = 3;
		m_invalidIndexLocalPosition.coordinate = SurgSim::Math::Vector::Zero(4);
		m_invalidIndexLocalPosition.coordinate[0] = 0.4;
		m_invalidIndexLocalPosition.coordinate[1] = 0.6;

		m_invalidCoordinateLocalPosition.index = 1;
		m_invalidCoordinateLocalPosition.coordinate = SurgSim::Math::Vector::Zero(4);
		m_invalidCoordinateLocalPosition.coordinate[0] = 0.6;
		m_invalidCoordinateLocalPosition.coordinate[1] = 0.6;
	}

	void TearDown()
	{
	}

	std::shared_ptr<Fem3DRepresentation> m_fem;
	std::shared_ptr<Fem3DRepresentation> m_fem3DCube;
	SurgSim::DataStructures::IndexedLocalCoordinate m_validLocalPosition;
	SurgSim::DataStructures::IndexedLocalCoordinate m_invalidIndexLocalPosition;
	SurgSim::DataStructures::IndexedLocalCoordinate m_invalidCoordinateLocalPosition;
	SurgSim::DataStructures::IndexedLocalCoordinate m_validLocalPositionForCube;
};

TEST_F(Fem3DRepresentationLocalizationTest, ConstructorTest)
{
	SurgSim::DataStructures::IndexedLocalCoordinate m_OneNodeValid, m_OneNodeInvalid;
	m_OneNodeValid.index = 0;
	m_OneNodeInvalid.index = 1000;

	ASSERT_NO_THROW(std::make_shared<Fem3DRepresentationLocalization>(m_fem, m_OneNodeValid));

	ASSERT_THROW(std::make_shared<Fem3DRepresentationLocalization>(m_fem, m_OneNodeInvalid),
		SurgSim::Framework::AssertionFailure);

	ASSERT_THROW(std::make_shared<Fem3DRepresentationLocalization>(m_fem, m_invalidIndexLocalPosition),
		SurgSim::Framework::AssertionFailure);

	ASSERT_THROW(std::make_shared<Fem3DRepresentationLocalization>(m_fem, m_invalidCoordinateLocalPosition),
		SurgSim::Framework::AssertionFailure);

	ASSERT_NO_THROW(std::make_shared<Fem3DRepresentationLocalization>(m_fem, m_validLocalPosition););
}

TEST_F(Fem3DRepresentationLocalizationTest, SetGetRepresentation)
{
	Fem3DRepresentationLocalization localization(m_fem, m_validLocalPosition);

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
	m_otherValidLocalPosition.coordinate = SurgSim::Math::Vector::Zero(4);
	m_otherValidLocalPosition.coordinate[1] = 1.0;

	localization.setLocalPosition(m_otherValidLocalPosition);
	EXPECT_EQ(m_otherValidLocalPosition.index, localization.getLocalPosition().index);
	EXPECT_TRUE(localization.getLocalPosition().coordinate.isApprox(m_otherValidLocalPosition.coordinate));
}

TEST_F(Fem3DRepresentationLocalizationTest, SetGetLocalization)
{
	using SurgSim::Math::Vector4d;
	using SurgSim::Math::Vector3d;

	{
		SCOPED_TRACE("Uninitialized Representation");

		// Uninitialized Representation
		EXPECT_THROW(std::make_shared<Fem3DRepresentationLocalization>(nullptr, m_validLocalPosition),
			SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Incorrectly formed natural coordinate");

		// Incorrectly formed natural coordinate
		auto localization = std::make_shared<Fem3DRepresentationLocalization>(m_fem, m_validLocalPosition);
		EXPECT_THROW(localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector4d(0.89, 0.54, 0.45, 0.64))),
			SurgSim::Framework::AssertionFailure);

		EXPECT_THROW(localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector3d(1.0, 0.0, 0.0))),
			SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Out of bounds element Id");

		// Out of bounds element Id
		auto localization = std::make_shared<Fem3DRepresentationLocalization>(m_fem, m_validLocalPosition);
		EXPECT_THROW(localization->setLocalPosition(IndexedLocalCoordinate(6u, Vector4d(1.0, 0.0, 0.0, 0.0))),
			SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("valid");

		auto localization = std::make_shared<Fem3DRepresentationLocalization>(m_fem, m_validLocalPosition);
		EXPECT_NO_THROW(localization->setLocalPosition(IndexedLocalCoordinate(1u, Vector4d(0.2, 0.6, 0.1, 0.1))));
		EXPECT_EQ(1u, localization->getLocalPosition().index);
		EXPECT_TRUE(localization->getLocalPosition().coordinate.isApprox(Vector4d(0.2, 0.6, 0.1, 0.1)));
	}
}

TEST_F(Fem3DRepresentationLocalizationTest, CalculatePositionTest)
{
	using SurgSim::Math::Vector;
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Vector4d;

	auto localization = std::make_shared<Fem3DRepresentationLocalization>(m_fem, m_validLocalPosition);

	// Test tetrahedron 1: nodes 0, 1, 2, 3
	localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector4d(1.0, 0.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 0.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector4d(0.0, 1.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector4d(0.0, 0.0, 1.0, 0.0)));
	EXPECT_TRUE(Vector3d(-1.0, 1.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector4d(0.0, 0.0, 0.0, 1.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	// Test tetrahedron 2: nodes 0, 1, 3, 4
	localization->setLocalPosition(IndexedLocalCoordinate(1u, Vector4d(1.0, 0.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 0.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(1u, Vector4d(0.0, 1.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(1u, Vector4d(0.0, 0.0, 1.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(1u, Vector4d(0.0, 0.0, 0.0, 1.0)));
	EXPECT_TRUE(Vector3d(1.0, 1.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	// Test tetrahedron 3: nodes 0, 1, 4, 5
	localization->setLocalPosition(IndexedLocalCoordinate(2u, Vector4d(1.0, 0.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 0.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(2u, Vector4d(0.0, 1.0, 0.0, 0.0)));
	EXPECT_TRUE(Vector3d(0.0, 1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(2u, Vector4d(0.0, 0.0, 1.0, 0.0)));
	EXPECT_TRUE(Vector3d(1.0, 1.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(2u, Vector4d(0.0, 0.0, 0.0, 1.0)));
	EXPECT_TRUE(Vector3d(1.0, 0.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	// Advanced tests
	localization->setLocalPosition(IndexedLocalCoordinate(0u, Vector4d(0.31, 0.03, 0.19, 0.47)));
	//   0.31 * ( 0.0,  0.0,  0.0) => ( 0.0,  0.0,   0.0 )
	// + 0.03 * ( 0.0,  1.0, -1.0) => ( 0.0,  0.03, -0.03)
	// + 0.19 * (-1.0,  1.0,  0.0) => (-0.19, 0.19,  0.0 )
	// + 0.47 * ( 0.0,  1.0,  0.0) => ( 0.0,  0.47,  0.0 )
	//                              = (-0.19, 0.69, -0.03)
	EXPECT_TRUE(Vector3d(-0.19, 0.69, -0.03).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(1u, Vector4d(0.05, 0.81, 0.06, 0.08)));
	//   0.05 * ( 0.0,  0.0,  0.0) => (0.0,  0.0,   0.0 )
	// + 0.81 * ( 0.0,  1.0, -1.0) => (0.0,  0.81, -0.81)
	// + 0.06 * ( 0.0,  1.0,  0.0) => (0.0,  0.06,  0.0 )
	// + 0.08 * ( 1.0,  1.0,  0.0) => (0.08, 0.08,  0.0 )
	//                              = (0.08, 0.95, -0.81)
	EXPECT_TRUE(Vector3d(0.08, 0.95, -0.81).isApprox(localization->calculatePosition(), epsilon));

	localization->setLocalPosition(IndexedLocalCoordinate(2u, Vector4d(0.11, 0.15, 0.67, 0.07)));
	//   0.11 * ( 0.0,  0.0,  0.0) => (0.0,  0.0,   0.0 )
	// + 0.15 * ( 0.0,  1.0, -1.0) => (0.0,  0.15, -0.15)
	// + 0.67 * ( 1.0,  1.0,  0.0) => (0.67, 0.67,  0.0 )
	// + 0.07 * ( 1.0,  0.0, -1.0) => (0.07, 0.0,  -0.07)
	//                              = (0.74, 0.82, -0.22)
	EXPECT_TRUE(Vector3d(0.74, 0.82, -0.22).isApprox(localization->calculatePosition(), epsilon));
}

TEST_F(Fem3DRepresentationLocalizationTest, CalculatePositionTest3DCube)
{
	auto localization = std::make_shared<Fem3DRepresentationLocalization>(m_fem3DCube, m_validLocalPositionForCube);
	SurgSim::Math::Vector cubeNodes(8);

	// Test central node
	cubeNodes << 0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125;
	localization->setLocalPosition(IndexedLocalCoordinate(0u, cubeNodes));
	EXPECT_TRUE(SurgSim::Math::Vector3d(0.0, 0.0, 0.0).isApprox(localization->calculatePosition(), epsilon));

	// Test node 0:
	cubeNodes << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	localization->setLocalPosition(IndexedLocalCoordinate(0u, cubeNodes));
	EXPECT_TRUE(SurgSim::Math::Vector3d(-1.0, -1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	// Test node 1:
	cubeNodes << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	localization->setLocalPosition(IndexedLocalCoordinate(0u, cubeNodes));
	EXPECT_TRUE(SurgSim::Math::Vector3d(1.0, -1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	// Test node 2:
	cubeNodes << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	localization->setLocalPosition(IndexedLocalCoordinate(0u, cubeNodes));
	EXPECT_TRUE(SurgSim::Math::Vector3d(1.0, 1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	// Test node 3:
	cubeNodes << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
	localization->setLocalPosition(IndexedLocalCoordinate(0u, cubeNodes));
	EXPECT_TRUE(SurgSim::Math::Vector3d(-1.0, 1.0, -1.0).isApprox(localization->calculatePosition(), epsilon));

	// Test node 4:
	cubeNodes << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
	localization->setLocalPosition(IndexedLocalCoordinate(0u, cubeNodes));
	EXPECT_TRUE(SurgSim::Math::Vector3d(-1.0, -1.0, 1.0).isApprox(localization->calculatePosition(), epsilon));

	// Test node 5:
	cubeNodes << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
	localization->setLocalPosition(IndexedLocalCoordinate(0u, cubeNodes));
	EXPECT_TRUE(SurgSim::Math::Vector3d(1.0, -1.0, 1.0).isApprox(localization->calculatePosition(), epsilon));

	// Test node 6:
	cubeNodes << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
	localization->setLocalPosition(IndexedLocalCoordinate(0u, cubeNodes));
	EXPECT_TRUE(SurgSim::Math::Vector3d(1.0, 1.0, 1.0).isApprox(localization->calculatePosition(), epsilon));

	// Test node 7:
	cubeNodes << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
	localization->setLocalPosition(IndexedLocalCoordinate(0u, cubeNodes));
	EXPECT_TRUE(SurgSim::Math::Vector3d(-1.0, 1.0, 1.0).isApprox(localization->calculatePosition(), epsilon));

	// Advantage test
	cubeNodes << 0.03, 0.09, 0.07, 0.05, 0.1, 0.13, 0.26, 0.27;
	localization->setLocalPosition(IndexedLocalCoordinate(0u, cubeNodes));
	EXPECT_TRUE(SurgSim::Math::Vector3d(0.1, 0.3, 0.52).isApprox(localization->calculatePosition(), epsilon));
	// 0.03 * (-1.0,-1.0,-1.0) => (-0.03, -0.03, -0.03)
	// 0.09 * ( 1.0,-1.0,-1.0) => ( 0.09, -0.09, -0.09)
	// 0.07 * ( 1.0, 1.0,-1.0) => ( 0.07,  0.07, -0.07)
	// 0.05 * (-1.0, 1.0,-1.0) => (-0.05,  0.05, -0.05)
	// 0.1  * (-1.0,-1.0, 1.0) => (-0.1,  -0.1,   0.1)
	// 0.13 * ( 1.0,-1.0, 1.0) => ( 0.13, -0.13,  0.13)
	// 0.26 * ( 1.0, 1.0, 1.0) => ( 0.26,  0.26,  0.26)
	// 0.27 * (-1.0, 1.0, 1.0) => (-0.27,  0.27,  0.27)
	//                         =  ( 0.1,   0.3,   0.52)
}

} // namespace SurgSim
} // namespace Physics

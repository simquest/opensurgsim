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
#include "SurgSim/Physics/Fem2DElementTriangle.h"
#include "SurgSim/Physics/Fem2DLocalization.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"

using SurgSim::DataStructures::IndexedLocalCoordinate;

namespace
{
const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Physics
{

void addTriangle(Fem2DRepresentation* fem, std::array<size_t, 3> nodes,
				 const SurgSim::Math::OdeState& state, double thickness = 0.01,
				 double massDensity = 1.0, double poissonRatio = 0.1, double youngModulus = 1.0)
{
	std::shared_ptr<Fem2DElementTriangle> element(new Fem2DElementTriangle(nodes));
	element->setThickness(thickness);
	element->setMassDensity(massDensity);
	element->setPoissonRatio(poissonRatio);
	element->setYoungModulus(youngModulus);
	element->initialize(state);
	fem->addFemElement(element);
}

class Fem2DLocalizationTest : public ::testing::Test
{
public:
	void SetUp()
	{
		using SurgSim::Math::Vector3d;
		using SurgSim::Math::getSubVector;

		m_fem = std::make_shared<Fem2DRepresentation>("Fem2dRepresentation");
		auto state = std::make_shared<SurgSim::Math::OdeState>();
		state->setNumDof(6, 4);

		auto& x = state->getPositions();
		getSubVector(x, 0, 6).segment<3>(0) = Vector3d(-1.0, 0.0, 0.0);
		getSubVector(x, 1, 6).segment<3>(0) = Vector3d(0.0, -1.0, 0.0);
		getSubVector(x, 2, 6).segment<3>(0) = Vector3d(1.0, 0.0, 0.0);
		getSubVector(x, 3, 6).segment<3>(0) = Vector3d(0.0, 1.0, 0.0);

		// Define Triangles
		{
			std::array<size_t, 3> nodes = {{0, 2, 1}};
			addTriangle(m_fem.get(), nodes, *state);
		}

		{
			std::array<size_t, 3> nodes = {{0, 3, 2}};
			addTriangle(m_fem.get(), nodes, *state);
		}

		m_fem->setInitialState(state);
		m_fem->setLocalActive(true);

		m_validLocalPosition.index = 1;
		m_validLocalPosition.coordinate = SurgSim::Math::Vector::Zero(3);
		m_validLocalPosition.coordinate[0] = 0.4;
		m_validLocalPosition.coordinate[1] = 0.6;

		m_invalidIndexLocalPosition.index = 3;
		m_invalidIndexLocalPosition.coordinate = SurgSim::Math::Vector::Zero(3);
		m_invalidIndexLocalPosition.coordinate[0] = 0.4;
		m_invalidIndexLocalPosition.coordinate[1] = 0.6;

		m_invalidCoordinateLocalPosition.index = 1;
		m_invalidCoordinateLocalPosition.coordinate = SurgSim::Math::Vector::Zero(3);
		m_invalidCoordinateLocalPosition.coordinate[0] = 0.6;
		m_invalidCoordinateLocalPosition.coordinate[1] = 0.6;
	}

	void TearDown()
	{
	}

	std::shared_ptr<Fem2DRepresentation> m_fem;
	SurgSim::DataStructures::IndexedLocalCoordinate m_validLocalPosition;
	SurgSim::DataStructures::IndexedLocalCoordinate m_invalidIndexLocalPosition;
	SurgSim::DataStructures::IndexedLocalCoordinate m_invalidCoordinateLocalPosition;
};

TEST_F(Fem2DLocalizationTest, ConstructorTest)
{
	// IndexedLocalCoordinate pointing to a node (node index + empty coordinate) are invalid. It will failed,
	// either because the index is out of bound or because the coordinates are the wrong size (empty)
	// This is tested by m_invalidIndexLocalPosition and m_invalidCoordinateLocalPosition
	ASSERT_THROW(std::make_shared<Fem2DLocalization>(m_fem, m_invalidIndexLocalPosition),
				 SurgSim::Framework::AssertionFailure);

	ASSERT_THROW(std::make_shared<Fem2DLocalization>(m_fem, m_invalidCoordinateLocalPosition),
				 SurgSim::Framework::AssertionFailure);

	ASSERT_NO_THROW(std::make_shared<Fem2DLocalization>(m_fem, m_validLocalPosition););
}

TEST_F(Fem2DLocalizationTest, IsValidRepresentation)
{
	Fem2DLocalization localization(m_fem, m_validLocalPosition);

	ASSERT_TRUE(localization.isValidRepresentation(m_fem));

	// nullptr is valid
	ASSERT_TRUE(localization.isValidRepresentation(nullptr));

	ASSERT_FALSE(localization.isValidRepresentation(std::make_shared<Fem1DRepresentation>("fem1d")));
	ASSERT_FALSE(localization.isValidRepresentation(std::make_shared<Fem3DRepresentation>("fem3d")));
}

TEST_F(Fem2DLocalizationTest, ElementPose)
{
	SurgSim::Math::RigidTransform3d pose0, pose1;
	SurgSim::Math::Vector3d edge0(1.0, 0.0, 0.0);
	SurgSim::Math::Vector3d edge1 = SurgSim::Math::Vector3d(1.0, 1.0, 0.0).normalized();
	SurgSim::Math::Vector3d normal(0.0, 0.0, 1.0);
	SurgSim::Math::Vector3d binormal0 = edge0.cross(normal);
	SurgSim::Math::Vector3d binormal1 = edge1.cross(normal);
	SurgSim::Math::Matrix33d rotation;
	rotation << edge0, normal, binormal0;
	pose0 = SurgSim::Math::makeRigidTransform(rotation, SurgSim::Math::Vector3d(0.0, -1.0 / 3.0, 0.0));
	rotation << edge1, normal, binormal1;
	pose1 = SurgSim::Math::makeRigidTransform(rotation, SurgSim::Math::Vector3d(0.0, 1.0 / 3.0, 0.0));

	{
		SurgSim::DataStructures::IndexedLocalCoordinate testPosition1;
		testPosition1.index = 0;
		testPosition1.coordinate = SurgSim::Math::Vector::Zero(3);
		testPosition1.coordinate[0] = 0.5;
		testPosition1.coordinate[1] = 0.5;
		Fem2DLocalization testLocalization1(m_fem, testPosition1);
		auto pose = testLocalization1.getElementPose();
		EXPECT_TRUE(pose0.isApprox(pose));
	}

	{
		SurgSim::DataStructures::IndexedLocalCoordinate testPosition1;
		testPosition1.index = 1;
		testPosition1.coordinate = SurgSim::Math::Vector::Zero(3);
		testPosition1.coordinate[0] = 0.5;
		testPosition1.coordinate[1] = 0.5;
		Fem2DLocalization testLocalization1(m_fem, testPosition1);
		auto pose = testLocalization1.getElementPose();
		EXPECT_TRUE(pose1.isApprox(pose));
	}
}

} // namespace SurgSim
} // namespace Physics

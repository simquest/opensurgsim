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
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/Fem3DElementCube.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"
#include "SurgSim/Physics/Fem3DLocalization.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"

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

void addTetraheadron(Fem3DRepresentation* fem, std::array<size_t, 4> nodes,
					 const SurgSim::Math::OdeState& state, double massDensity = 1.0,
					 double poissonRatio = 0.1, double youngModulus = 1.0)
{
	std::shared_ptr<Fem3DElementTetrahedron> element(new Fem3DElementTetrahedron(nodes));
	element->setMassDensity(massDensity);
	element->setPoissonRatio(poissonRatio);
	element->setYoungModulus(youngModulus);
	element->initialize(state);
	fem->addFemElement(element);
}

class Fem3DLocalizationTest : public ::testing::Test
{
public:
	void SetUp()
	{
		using SurgSim::Math::Vector3d;

		m_fem = std::make_shared<Fem3DRepresentation>("Fem3dRepresentation");
		auto state = std::make_shared<SurgSim::Math::OdeState>();
		state->setNumDof(3, 6);

		auto& x = state->getPositions();
		getSubVector(x, 0, 3) = Vector3d(0.0,  0.0,  0.0);
		getSubVector(x, 1, 3) = Vector3d(0.0,  1.0, -1.0);
		getSubVector(x, 2, 3) = Vector3d(-1.0,  1.0,  0.0);
		getSubVector(x, 3, 3) = Vector3d(0.0,  1.0,  0.0);
		getSubVector(x, 4, 3) = Vector3d(1.0,  1.0,  0.0);
		getSubVector(x, 5, 3) = Vector3d(1.0,  0.0, -1.0);

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
		getSubVector(x0, 0, 3) = Vector3d(-1.0, -1.0, -1.0);
		getSubVector(x0, 1, 3) = Vector3d(1.0, -1.0, -1.0);
		getSubVector(x0, 2, 3) = Vector3d(-1.0, 1.0, -1.0);
		getSubVector(x0, 3, 3) = Vector3d(1.0, 1.0, -1.0);
		getSubVector(x0, 4, 3) = Vector3d(-1.0, -1.0, 1.0);
		getSubVector(x0, 5, 3) = Vector3d(1.0, -1.0, 1.0);
		getSubVector(x0, 6, 3) = Vector3d(-1.0, 1.0, 1.0);
		getSubVector(x0, 7, 3) = Vector3d(1.0, 1.0, 1.0);

		// Define Cube
		{
			std::array<size_t, 8> node0 = {{0, 1, 3, 2, 4, 5, 7, 6}};
			std::shared_ptr<Fem3DElementCube> femElement(new Fem3DElementCube(node0));
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

TEST_F(Fem3DLocalizationTest, ConstructorTest)
{
	// IndexedLocalCoordinate pointing to a node (node index + empty coordinate) or a triangle
	// (triangleID + coordinate of size 2) are invalid. Both will failed, either because the index is out of bound,
	// or because the coordinates are the wrong size (empty or of size 2 do not correspond to any valid 3d FemElements)
	// This is tested by m_invalidIndexLocalPosition and m_invalidCoordinateLocalPosition
	ASSERT_THROW(std::make_shared<Fem3DLocalization>(m_fem, m_invalidIndexLocalPosition),
				 SurgSim::Framework::AssertionFailure);

	ASSERT_THROW(std::make_shared<Fem3DLocalization>(m_fem, m_invalidCoordinateLocalPosition),
				 SurgSim::Framework::AssertionFailure);

	ASSERT_NO_THROW(std::make_shared<Fem3DLocalization>(m_fem, m_validLocalPosition););

	ASSERT_NO_THROW(std::make_shared<Fem3DLocalization>(m_fem3DCube, m_validLocalPositionForCube););
}

TEST_F(Fem3DLocalizationTest, IsValidRepresentation)
{
	Fem3DLocalization localization(m_fem, m_validLocalPosition);

	ASSERT_TRUE(localization.isValidRepresentation(m_fem));

	// nullptr is valid
	ASSERT_TRUE(localization.isValidRepresentation(nullptr));

	ASSERT_FALSE(localization.isValidRepresentation(std::make_shared<Fem1DRepresentation>("fem1d")));
	ASSERT_FALSE(localization.isValidRepresentation(std::make_shared<Fem2DRepresentation>("fem2d")));
}

} // namespace SurgSim
} // namespace Physics

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

#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/Fem2DElementTriangle.h"
#include "SurgSim/Physics/Fem3DElementCorotationalTetrahedron.h"
#include "SurgSim/Physics/Fem3DElementCube.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::Physics::FemElement;
using SurgSim::Math::Vector;
using SurgSim::Math::Matrix;

void testSize(const Vector& v, size_t expectedSize)
{
	EXPECT_EQ(expectedSize, v.size());
}

void testSize(const Matrix& m, size_t expectedRows, size_t expectedCols)
{
	EXPECT_EQ(expectedRows, m.rows());
	EXPECT_EQ(expectedCols, m.cols());
}

namespace SurgSim
{

namespace Physics
{

TEST(FemElementTests, GetSetAddMethods)
{
	MockFemElement femElement;

	// Initial setup (numDofPerNode set), no nodes defined yet, density = 0
	EXPECT_EQ(3u, femElement.getNumDofPerNode());
	EXPECT_EQ(0u, femElement.getNumNodes());
	EXPECT_EQ(0, femElement.getNodeIds().size());
	EXPECT_DOUBLE_EQ(0.0, femElement.getMassDensity());
	EXPECT_DOUBLE_EQ(0.0, femElement.getYoungModulus());
	EXPECT_DOUBLE_EQ(0.0, femElement.getPoissonRatio());

	// Test Set/Get Young modulus
	femElement.setYoungModulus(4455.33);
	EXPECT_DOUBLE_EQ(4455.33, femElement.getYoungModulus());
	femElement.setYoungModulus(0.0);
	EXPECT_DOUBLE_EQ(0.0, femElement.getYoungModulus());

	// Test Set/Get Poisson ratio
	femElement.setPoissonRatio(0.45);
	EXPECT_DOUBLE_EQ(0.45, femElement.getPoissonRatio());
	femElement.setPoissonRatio(0.0);
	EXPECT_DOUBLE_EQ(0.0, femElement.getPoissonRatio());

	// Test Set/Get mass density
	femElement.setMassDensity(2343.13);
	EXPECT_DOUBLE_EQ(2343.13, femElement.getMassDensity());
	femElement.setMassDensity(0.0);
	EXPECT_DOUBLE_EQ(0.0, femElement.getMassDensity());

	// Test GetMass
	SurgSim::Math::OdeState fakeState;
	femElement.setMassDensity(0.0);
	EXPECT_DOUBLE_EQ(0.0, femElement.getMass(fakeState));
	femElement.setMassDensity(1.14);
	EXPECT_DOUBLE_EQ(1.14, femElement.getMass(fakeState));
	femElement.setMassDensity(434.55);
	EXPECT_DOUBLE_EQ(434.55, femElement.getMass(fakeState));

	// Add 1 node
	femElement.addNode(0);
	EXPECT_EQ(3u, femElement.getNumDofPerNode());
	EXPECT_EQ(1u, femElement.getNumNodes());
	EXPECT_EQ(1, femElement.getNodeIds().size());
	EXPECT_EQ(0, femElement.getNodeIds()[0]);
	EXPECT_EQ(0, femElement.getNodeId(0));

	// Add 1 more node
	femElement.addNode(9);
	EXPECT_EQ(3u, femElement.getNumDofPerNode());
	EXPECT_EQ(2u, femElement.getNumNodes());
	EXPECT_EQ(2, femElement.getNodeIds().size());
	EXPECT_EQ(0, femElement.getNodeIds()[0]);
	EXPECT_EQ(0, femElement.getNodeId(0));
	EXPECT_EQ(9, femElement.getNodeIds()[1]);
	EXPECT_EQ(9, femElement.getNodeId(1));
}

TEST(FemElementTests, InitializeMethods)
{
	MockFemElement femElement;
	SurgSim::Math::OdeState fakeState;

	// Mass density not set
	ASSERT_ANY_THROW(femElement.initialize(fakeState));

	// Poisson Ratio not set
	femElement.setMassDensity(-1234.56);
	ASSERT_ANY_THROW(femElement.initialize(fakeState));

	// Young modulus not set
	femElement.setPoissonRatio(0.55);
	ASSERT_ANY_THROW(femElement.initialize(fakeState));

	// Invalid mass density
	femElement.setYoungModulus(-4321.33);
	ASSERT_ANY_THROW(femElement.initialize(fakeState));

	// Invalid Poisson ratio
	femElement.setMassDensity(1234.56);
	ASSERT_ANY_THROW(femElement.initialize(fakeState));

	// Invalid Young modulus
	femElement.setPoissonRatio(0.499);
	ASSERT_ANY_THROW(femElement.initialize(fakeState));

	femElement.setYoungModulus(4321.33);
	ASSERT_NO_THROW(femElement.initialize(fakeState));
}

void checkValidCoordinate(const MockFemElement& femElement, double v0, bool expected)
{
	Vector naturalCoordinate(1);
	naturalCoordinate << v0;
	EXPECT_EQ(expected, femElement.isValidCoordinate(naturalCoordinate));
}

void checkValidCoordinate(const MockFemElement& femElement, double v0, double v1, bool expected)
{
	Vector naturalCoordinate(2);
	naturalCoordinate << v0, v1;
	EXPECT_EQ(expected, femElement.isValidCoordinate(naturalCoordinate));
}

void checkValidCoordinate(const MockFemElement& femElement, double v0, double v1, double v2, bool expected)
{
	Vector naturalCoordinate(3);
	naturalCoordinate << v0, v1, v2;
	EXPECT_EQ(expected, femElement.isValidCoordinate(naturalCoordinate));
}

void checkValidCoordinate(const MockFemElement& femElement, double v0, double v1, double v2, double v3, bool expected)
{
	Vector naturalCoordinate(4);
	naturalCoordinate << v0, v1, v2, v3;
	EXPECT_EQ(expected, femElement.isValidCoordinate(naturalCoordinate));
}

TEST(FemElementTests, IsValidCoordinate)
{
	MockFemElement femElement;
	femElement.addNode(0);
	double e = 1e-11;

	checkValidCoordinate(femElement, 1.0, true);
	checkValidCoordinate(femElement, 1.0 + e, true);
	checkValidCoordinate(femElement, 1.0 - e, true);
	checkValidCoordinate(femElement, 1.01, false);
	checkValidCoordinate(femElement, -1.01, false);
	checkValidCoordinate(femElement, 0.7, false);

	femElement.addNode(1);

	checkValidCoordinate(femElement, 1.0, 0.0, true);
	checkValidCoordinate(femElement, 1.0 + e, 0.0, true);
	checkValidCoordinate(femElement, 1.0 + e, 0.0 - e, true);
	checkValidCoordinate(femElement, 1.0 - e, 0.0 + e, true);
	checkValidCoordinate(femElement, 0.5, 0.5, true);
	checkValidCoordinate(femElement, 0.5 + e, 0.5 + e, true);
	checkValidCoordinate(femElement, 0.5, 0.51, false);
	checkValidCoordinate(femElement, 1.0, false);
	checkValidCoordinate(femElement, -0.01, 1.01, false);

	femElement.addNode(2);

	checkValidCoordinate(femElement, 1.0, 0.0, 0.0, true);
	checkValidCoordinate(femElement, 1.0 + e, 0.0, 0.0, true);
	checkValidCoordinate(femElement, 1.0 + e, 0.0 - e, 0.0, true);
	checkValidCoordinate(femElement, 1.0 - e, 0.0 + e, e, true);
	checkValidCoordinate(femElement, 0.5, 0.5, e, true);
	checkValidCoordinate(femElement, 0.5 + e, 0.5 + e, -e, true);
	checkValidCoordinate(femElement, 0.5, 0.41, 0.1, false);
	checkValidCoordinate(femElement, 1.0, 0.0, false);
	checkValidCoordinate(femElement, -0.01, 1.01, e, false);

	femElement.addNode(3);

	checkValidCoordinate(femElement, 1.0, 0.0, 0.0, 0.0, true);
	checkValidCoordinate(femElement, 1.0 + e, 0.0, 0.0, 0.0, true);
	checkValidCoordinate(femElement, 1.0 + e, 0.0 - e, 0.0, 0.0, true);
	checkValidCoordinate(femElement, 1.0 - e, 0.0 + e, e, 0.0, true);
	checkValidCoordinate(femElement, 0.5, 0.5, e, 0.0, true);
	checkValidCoordinate(femElement, 0.5 + e, 0.5 + e, 0.0, -e, true);
	checkValidCoordinate(femElement, 0.5, 0.0, 0.41, 0.1, false);
	checkValidCoordinate(femElement, 0.0, 1.0, 0.0, false);
	checkValidCoordinate(femElement, -0.01, 0.0, 1.01, e, false);
}

TEST(FemElementTests, FactoryTest)
{
	std::vector<size_t> mockNodes;
	mockNodes.push_back(2);
	FemElement::getFactory().registerClass<MockFemElement>("MockFemElement");
	auto mockFem = FemElement::getFactory().create("MockFemElement", mockNodes);
	EXPECT_NE(nullptr, mockFem);

	std::vector<size_t> beamNodes;
	beamNodes.push_back(1);
	beamNodes.push_back(2);
	auto beamFem = FemElement::getFactory().create("SurgSim::Physics::Fem1DElementBeam", beamNodes);
	EXPECT_NE(nullptr, beamFem);
	ASSERT_ANY_THROW(FemElement::getFactory().create("SurgSim::Physics::Fem1DElementBeam", mockNodes));

	std::vector<size_t> triNodes;
	triNodes.push_back(1);
	triNodes.push_back(2);
	triNodes.push_back(3);
	auto triFem = FemElement::getFactory().create("SurgSim::Physics::Fem2DElementTriangle", triNodes);
	EXPECT_NE(nullptr, triFem);
	ASSERT_ANY_THROW(FemElement::getFactory().create("SurgSim::Physics::Fem2DElementTriangle", beamNodes));

	std::vector<size_t> tetNodes;
	tetNodes.push_back(1);
	tetNodes.push_back(2);
	tetNodes.push_back(3);
	tetNodes.push_back(1);
	auto coTetFem = FemElement::getFactory().create("SurgSim::Physics::Fem3DElementCorotationalTetrahedron", tetNodes);
	EXPECT_NE(nullptr, coTetFem);
	ASSERT_ANY_THROW(FemElement::getFactory().create(
		"SurgSim::Physics::Fem3DElementCorotationalTetrahedron", triNodes));

	std::vector<size_t> cubeNodes;
	cubeNodes.push_back(1);
	cubeNodes.push_back(2);
	cubeNodes.push_back(3);
	cubeNodes.push_back(4);
	cubeNodes.push_back(5);
	cubeNodes.push_back(6);
	cubeNodes.push_back(7);
	cubeNodes.push_back(8);
	auto cubeFem = FemElement::getFactory().create("SurgSim::Physics::Fem3DElementCube", cubeNodes);
	EXPECT_NE(nullptr, cubeFem);
	ASSERT_ANY_THROW(FemElement::getFactory().create("SurgSim::Physics::Fem3DElementCube", tetNodes));

	auto tetFem = FemElement::getFactory().create("SurgSim::Physics::Fem3DElementTetrahedron", tetNodes);
	EXPECT_NE(nullptr, tetFem);
	ASSERT_ANY_THROW(FemElement::getFactory().create("SurgSim::Physics::Fem3DElementTetrahedron", cubeNodes));
}

} // namespace Physics

} // namespace SurgSim

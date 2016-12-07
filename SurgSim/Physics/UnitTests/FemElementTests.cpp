// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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
	EXPECT_EQ(static_cast<Vector::Index>(expectedSize), v.size());
}

void testSize(const Matrix& m, size_t expectedRows, size_t expectedCols)
{
	EXPECT_EQ(static_cast<Matrix::Index>(expectedRows), m.rows());
	EXPECT_EQ(static_cast<Matrix::Index>(expectedCols), m.cols());
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
	EXPECT_EQ(0u, femElement.getNodeIds().size());
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
	EXPECT_EQ(1u, femElement.getNodeIds().size());
	EXPECT_EQ(0u, femElement.getNodeIds()[0]);
	EXPECT_EQ(0u, femElement.getNodeId(0));

	// Add 1 more node
	femElement.addNode(9);
	EXPECT_EQ(3u, femElement.getNumDofPerNode());
	EXPECT_EQ(2u, femElement.getNumNodes());
	EXPECT_EQ(2u, femElement.getNodeIds().size());
	EXPECT_EQ(0u, femElement.getNodeIds()[0]);
	EXPECT_EQ(0u, femElement.getNodeId(0));
	EXPECT_EQ(9u, femElement.getNodeIds()[1]);
	EXPECT_EQ(9u, femElement.getNodeId(1));
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
	auto mockElement = std::make_shared<FemElementStructs::FemElementParameter>();
	mockElement->nodeIds.push_back(2);
	FemElement::getFactory().registerClass<MockFemElement>("MockFemElement");

	// Test with a mock FemElement
	auto mockFem = FemElement::getFactory().create("MockFemElement", mockElement);
	EXPECT_NE(nullptr, mockFem);
	EXPECT_NE(nullptr, std::dynamic_pointer_cast<MockFemElement>(mockFem));

	// Test with a 1D beam
	auto beamElement = std::make_shared<FemElementStructs::FemElement1DParameter>();
	beamElement->nodeIds.push_back(1);
	beamElement->nodeIds.push_back(2);
	beamElement->radius = 0.4;
	beamElement->enableShear = false;
	beamElement->massDensity = 0.4;
	beamElement->poissonRatio = 0.4;
	beamElement->youngModulus = 0.4;
	static SurgSim::Physics::Fem1DElementBeam beam;
	auto beamFem = FemElement::getFactory().create(beam.getClassName(), beamElement);
	EXPECT_NE(nullptr, beamFem);
	EXPECT_NE(nullptr, std::dynamic_pointer_cast<Fem1DElementBeam>(beamFem));
	ASSERT_ANY_THROW(FemElement::getFactory().create("SurgSim::Physics::Fem1DElementBeam", mockElement));

	// Test with a 2D triangle
	auto triElement = std::make_shared<FemElementStructs::FemElement2DParameter>();
	triElement->nodeIds.push_back(1);
	triElement->nodeIds.push_back(2);
	triElement->nodeIds.push_back(3);
	triElement->thickness = 0.4;
	triElement->massDensity = 0.4;
	triElement->poissonRatio = 0.4;
	triElement->youngModulus = 0.4;
	static SurgSim::Physics::Fem2DElementTriangle triangle;
	auto triFem = FemElement::getFactory().create(triangle.getClassName(), triElement);
	EXPECT_NE(nullptr, triFem);
	EXPECT_NE(nullptr, std::dynamic_pointer_cast<Fem2DElementTriangle>(triFem));
	ASSERT_ANY_THROW(FemElement::getFactory().create("SurgSim::Physics::Fem2DElementTriangle", beamElement));

	// Test with a 3D corotational tetrahedron
	auto tetElement = std::make_shared<FemElementStructs::FemElement3DParameter>();
	tetElement->nodeIds.push_back(1);
	tetElement->nodeIds.push_back(2);
	tetElement->nodeIds.push_back(3);
	tetElement->nodeIds.push_back(1);
	tetElement->massDensity = 0.4;
	tetElement->poissonRatio = 0.4;
	tetElement->youngModulus = 0.4;
	static SurgSim::Physics::Fem3DElementCorotationalTetrahedron corotationalTetrahedron;
	auto coTetFem = FemElement::getFactory().create(
		corotationalTetrahedron.getClassName(), tetElement);
	EXPECT_NE(nullptr, coTetFem);
	EXPECT_NE(nullptr, std::dynamic_pointer_cast<Fem3DElementCorotationalTetrahedron>(coTetFem));
	ASSERT_ANY_THROW(FemElement::getFactory().create(
		"SurgSim::Physics::Fem3DElementCorotationalTetrahedron", triElement));

	// Test with a 3D cube
	auto cubeElement = std::make_shared<FemElementStructs::FemElement3DParameter>();
	cubeElement->nodeIds.push_back(1);
	cubeElement->nodeIds.push_back(2);
	cubeElement->nodeIds.push_back(3);
	cubeElement->nodeIds.push_back(4);
	cubeElement->nodeIds.push_back(5);
	cubeElement->nodeIds.push_back(6);
	cubeElement->nodeIds.push_back(7);
	cubeElement->nodeIds.push_back(8);
	cubeElement->massDensity = 0.4;
	cubeElement->poissonRatio = 0.4;
	cubeElement->youngModulus = 0.4;
	static SurgSim::Physics::Fem3DElementCube cube;
	auto cubeFem = FemElement::getFactory().create(cube.getClassName(), cubeElement);
	EXPECT_NE(nullptr, cubeFem);
	EXPECT_NE(nullptr, std::dynamic_pointer_cast<Fem3DElementCube>(cubeFem));
	ASSERT_ANY_THROW(FemElement::getFactory().create(cube.getClassName(), tetElement));

	// Test with a 3D tetrahedron
	static SurgSim::Physics::Fem3DElementTetrahedron tetrahedron;
	auto tetFem = FemElement::getFactory().create(tetrahedron.getClassName(), tetElement);
	EXPECT_NE(nullptr, tetFem);
	EXPECT_NE(nullptr, std::dynamic_pointer_cast<Fem3DElementTetrahedron>(tetFem));
	ASSERT_ANY_THROW(FemElement::getFactory().create(tetrahedron.getClassName(), cubeElement));
}

} // namespace Physics

} // namespace SurgSim

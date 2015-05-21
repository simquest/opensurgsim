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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/Fem2DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/Fem2DElementTriangle.h"

namespace SurgSim
{
namespace Physics
{
using SurgSim::Math::Vector3d;
using SurgSim::DataStructures::PlyReader;

TEST(Fem2DRepresentationReaderTests, DelegateTest)
{
	auto femRepresentation = std::make_shared<Fem2DRepresentation>("Representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	femRepresentation->loadMesh("PlyReaderTests/Fem2D.ply");
	ASSERT_TRUE(femRepresentation->initialize(runtime));

	// Vertices
	ASSERT_EQ(6u, femRepresentation->getNumDofPerNode());
	ASSERT_EQ(6u * 6u, femRepresentation->getNumDof());

	Vector3d vertex0(1.0, 1.0, -1.0);
	Vector3d vertex5(0.999999, -1.000001, 1.0);

	EXPECT_TRUE(vertex0.isApprox(femRepresentation->getInitialState()->getPosition(0)));
	EXPECT_TRUE(vertex5.isApprox(femRepresentation->getInitialState()->getPosition(5)));

	// Number of triangles
	ASSERT_EQ(3u, femRepresentation->getNumFemElements());

	std::array<size_t, 3> triangle0 = {0, 1, 2};
	std::array<size_t, 3> triangle2 = {3, 4, 5};

	EXPECT_TRUE(std::equal(std::begin(triangle0), std::end(triangle0),
						   std::begin(femRepresentation->getFemElement(0)->getNodeIds())));
	EXPECT_TRUE(std::equal(std::begin(triangle2), std::end(triangle2),
						   std::begin(femRepresentation->getFemElement(2)->getNodeIds())));

	// Boundary conditions
	ASSERT_EQ(2u * 6u, femRepresentation->getInitialState()->getNumBoundaryConditions());

	// Boundary condition 0 is on node 8
	size_t boundaryNode0 = 3;
	size_t boundaryNode1 = 2;

	EXPECT_EQ(6 * boundaryNode0, femRepresentation->getInitialState()->getBoundaryConditions().at(0));
	EXPECT_EQ(6 * boundaryNode0 + 1, femRepresentation->getInitialState()->getBoundaryConditions().at(1));
	EXPECT_EQ(6 * boundaryNode0 + 2, femRepresentation->getInitialState()->getBoundaryConditions().at(2));
	EXPECT_EQ(6 * boundaryNode1, femRepresentation->getInitialState()->getBoundaryConditions().at(6));
	EXPECT_EQ(6 * boundaryNode1 + 1, femRepresentation->getInitialState()->getBoundaryConditions().at(7));
	EXPECT_EQ(6 * boundaryNode1 + 2, femRepresentation->getInitialState()->getBoundaryConditions().at(8));

	// Material
	for (size_t i = 0; i < femRepresentation->getNumFemElements(); ++i)
	{
		auto fem = femRepresentation->getFemElement(i);
		EXPECT_DOUBLE_EQ(0.2, fem->getMassDensity());
		EXPECT_DOUBLE_EQ(0.3, fem->getPoissonRatio());
		EXPECT_DOUBLE_EQ(0.4, fem->getYoungModulus());

		auto fem2DTriganle = std::dynamic_pointer_cast<SurgSim::Physics::Fem2DElementTriangle>(fem);
		ASSERT_NE(nullptr, fem2DTriganle);
		EXPECT_DOUBLE_EQ(0.1, fem2DTriganle->getThickness());
	}
}

}; // Physics
}; // SurgSim

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
#include "SurgSim/Physics/Fem1DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"

namespace SurgSim
{
namespace Physics
{
using SurgSim::Math::Vector3d;
using SurgSim::DataStructures::PlyReader;

TEST(Fem1DRepresentationReaderTests, DelegateTest)
{
	auto femRepresentation = std::make_shared<Fem1DRepresentation>("Representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	femRepresentation->loadMesh("PlyReaderTests/Fem1D.ply");
	ASSERT_TRUE(femRepresentation->initialize(runtime));

	// Vertices
	ASSERT_EQ(6u, femRepresentation->getNumDofPerNode());
	ASSERT_EQ(6u * 7u, femRepresentation->getNumDof());

	Vector3d vertex0(1.1, 1.2, -1.3);
	Vector3d vertex6(9.100000, 9.200000, 9.300000);

	EXPECT_TRUE(vertex0.isApprox(femRepresentation->getInitialState()->getPosition(0)));
	EXPECT_TRUE(vertex6.isApprox(femRepresentation->getInitialState()->getPosition(6)));

	// Number of beams
	ASSERT_EQ(4u, femRepresentation->getNumFemElements());

	std::array<size_t, 2> beam0 = {0, 1};
	std::array<size_t, 2> beam2 = {3, 5};

	EXPECT_TRUE(std::equal(std::begin(beam0), std::end(beam0),
						   std::begin(femRepresentation->getFemElement(0)->getNodeIds())));
	EXPECT_TRUE(std::equal(std::begin(beam2), std::end(beam2),
						   std::begin(femRepresentation->getFemElement(2)->getNodeIds())));

	// Boundary conditions
	ASSERT_EQ(3u * 6u, femRepresentation->getInitialState()->getNumBoundaryConditions());

	// Boundary condition 0 is on node 8
	size_t boundaryNode0 = 2;
	size_t boundaryNode2 = 5;

	EXPECT_EQ(6 * boundaryNode0, femRepresentation->getInitialState()->getBoundaryConditions().at(0));
	EXPECT_EQ(6 * boundaryNode0 + 1, femRepresentation->getInitialState()->getBoundaryConditions().at(1));
	EXPECT_EQ(6 * boundaryNode0 + 2, femRepresentation->getInitialState()->getBoundaryConditions().at(2));
	EXPECT_EQ(6 * boundaryNode2, femRepresentation->getInitialState()->getBoundaryConditions().at(12));
	EXPECT_EQ(6 * boundaryNode2 + 1, femRepresentation->getInitialState()->getBoundaryConditions().at(13));
	EXPECT_EQ(6 * boundaryNode2 + 2, femRepresentation->getInitialState()->getBoundaryConditions().at(14));

	// Material
	for (size_t i = 0; i < femRepresentation->getNumFemElements(); ++i)
	{
		auto fem = femRepresentation->getFemElement(i);
		EXPECT_DOUBLE_EQ(0.21, fem->getMassDensity());
		EXPECT_DOUBLE_EQ(0.31, fem->getPoissonRatio());
		EXPECT_DOUBLE_EQ(0.41, fem->getYoungModulus());

		auto fem1DBeam = std::dynamic_pointer_cast<SurgSim::Physics::Fem1DElementBeam>(fem);
		ASSERT_NE(nullptr, fem1DBeam);
		EXPECT_DOUBLE_EQ(0.11, fem1DBeam->getRadius());
	}
}

}; // Physics
}; // SurgSim

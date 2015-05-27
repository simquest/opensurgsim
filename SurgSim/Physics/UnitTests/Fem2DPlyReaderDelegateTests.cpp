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
#include "SurgSim/Physics/Fem.h"
#include "SurgSim/Physics/Fem2DPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
using SurgSim::Math::Vector3d;
using SurgSim::DataStructures::PlyReader;

TEST(Fem2DRepresentationReaderTests, DelegateTest)
{
	auto fem = std::make_shared<Fem2D>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	fem->load("PlyReaderTests/Fem2D.ply");

	// Vertices
	Vector3d vertex0(1.0, 1.0, -1.0);
	Vector3d vertex5(0.999999, -1.000001, 1.0);

	EXPECT_TRUE(vertex0.isApprox(fem->getVertex(0).position));
	EXPECT_TRUE(vertex5.isApprox(fem->getVertex(5).position));

	// Number of triangles
	ASSERT_EQ(3u, fem->getNumElements());

	std::array<size_t, 3> triangle0 = {0, 1, 2};
	std::array<size_t, 3> triangle2 = {3, 4, 5};

	EXPECT_TRUE(std::equal(std::begin(triangle0), std::end(triangle0),
						   std::begin(fem->getFemElement(0)->nodeIds)));
	EXPECT_TRUE(std::equal(std::begin(triangle2), std::end(triangle2),
						   std::begin(fem->getFemElement(2)->nodeIds)));

	// Boundary conditions
	ASSERT_EQ(2u, fem->getBoundaryConditions().size());

	EXPECT_EQ(3, fem->getBoundaryCondition(0));
	EXPECT_EQ(2, fem->getBoundaryCondition(1));

	// Material
	for (size_t i = 0; i < fem->getNumElements(); ++i)
	{
		auto element = fem->getFemElement(i);
		EXPECT_DOUBLE_EQ(0.2, element->massDensity);
		EXPECT_DOUBLE_EQ(0.3, element->poissonRatio);
		EXPECT_DOUBLE_EQ(0.4, element->youngModulus);
	}
}

}; // Physics
}; // SurgSim

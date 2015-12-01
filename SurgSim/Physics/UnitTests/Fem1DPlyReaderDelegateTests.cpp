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
#include "SurgSim/Physics/Fem1DPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
using SurgSim::Math::Vector3d;
using SurgSim::DataStructures::PlyReader;

TEST(Fem1DRepresentationReaderTests, DelegateTest)
{
	auto fem = std::make_shared<Fem1D>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	fem->load("PlyReaderTests/Fem1D.ply");

	// Vertices
	Vector3d vertex0(1.1, 1.2, -1.3);
	Vector3d vertex6(9.100000, 9.200000, 9.300000);

	EXPECT_TRUE(vertex0.isApprox(fem->getVertex(0).position));
	EXPECT_TRUE(vertex6.isApprox(fem->getVertex(6).position));

	// Number of beams
	ASSERT_EQ(4u, fem->getNumElements());

	std::array<size_t, 2> beam0 = {0, 1};
	std::array<size_t, 2> beam2 = {3, 5};

	EXPECT_TRUE(std::equal(std::begin(beam0), std::end(beam0),
						   std::begin(fem->getElement(0)->nodeIds)));
	EXPECT_TRUE(std::equal(std::begin(beam2), std::end(beam2),
						   std::begin(fem->getElement(2)->nodeIds)));

	// Boundary conditions
	ASSERT_EQ(3u, fem->getBoundaryConditions().size());

	EXPECT_EQ(2, fem->getBoundaryCondition(0));
	EXPECT_EQ(4, fem->getBoundaryCondition(1));
	EXPECT_EQ(5, fem->getBoundaryCondition(2));

	// Material
	for (size_t i = 0; i < fem->getNumElements(); ++i)
	{
		auto element = fem->getElement(i);
		EXPECT_DOUBLE_EQ(0.21, element->massDensity);
		EXPECT_DOUBLE_EQ(0.31, element->poissonRatio);
		EXPECT_DOUBLE_EQ(0.41, element->youngModulus);
	}
}

TEST(Fem1DRepresentationReaderTests, PerElementMaterial)
{
	auto fem = std::make_shared<Fem1D>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	fem->load("PlyReaderTests/Fem1DMaterial.ply");

	// Material
	double value = 1.0;
	for (size_t i = 0; i < fem->getNumElements(); ++i)
	{
		auto element = fem->getElement(i);
		EXPECT_DOUBLE_EQ(value++, element->massDensity);
		EXPECT_DOUBLE_EQ(value++, element->poissonRatio);
		EXPECT_DOUBLE_EQ(value++, element->youngModulus);
	}
}

}; // Physics
}; // SurgSim

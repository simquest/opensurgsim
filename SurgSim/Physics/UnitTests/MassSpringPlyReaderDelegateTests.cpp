// This file is a part of the OpenSurgSim project.
// Copyright 2020, SimQuest Solutions Inc.
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
#include "SurgSim/Physics/MassSpringPlyReaderDelegate.h"
#include "SurgSim/Physics/MassSpringModel.h"
#include "SurgSim/Physics/Spring.h"

namespace SurgSim
{
namespace Physics
{
using SurgSim::Math::Vector3d;
using SurgSim::DataStructures::PlyReader;

TEST(MassSpringPlyReaderDelegateTests, DelegateTest1D)
{
	auto mesh = std::make_shared<MassSpringModel>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	mesh->load("PlyReaderTests/MassSpring1D.ply");
	// Vertices
	Vector3d vertex0(-0.020006, 0.014949, 0.00004);
	Vector3d vertex6(0.00018, 0.0002, 0);

	EXPECT_TRUE(vertex0.isApprox(mesh->getVertex(0).position));
	EXPECT_TRUE(vertex6.isApprox(mesh->getVertex(6).position));
	EXPECT_NEAR(mesh->getMass(6)->getMass(), 2.64118e-05, 1e-6);

	ASSERT_EQ(mesh->getNumElements(), 65);
	ASSERT_EQ(mesh->getNodeIds(0).size(), 2);
	std::vector<size_t> nodeIds = { 0, 1 };
	EXPECT_TRUE(std::equal(std::begin(nodeIds), std::end(nodeIds),
		std::begin(mesh->getNodeIds(0))));

	ASSERT_EQ(129u, mesh->getNumSprings());
	std::array<size_t, 2> spring0 = { 0, 1 };
	std::array<size_t, 2> spring2 = { 2, 3 };
	EXPECT_TRUE(std::equal(std::begin(spring0), std::end(spring0),
		std::begin(mesh->getSpring(0)->getNodeIds())));
	EXPECT_TRUE(std::equal(std::begin(spring2), std::end(spring2),
		std::begin(mesh->getSpring(2)->getNodeIds())));

	// Boundary conditions
	ASSERT_EQ(10u, mesh->getBoundaryConditions().size());
}
TEST(MassSpringPlyReaderDelegateTests, DelegateTest2D)
{
	auto mesh = std::make_shared<MassSpringModel>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	mesh->load("PlyReaderTests/MassSpring2D.ply");
	// Vertices
	Vector3d vertex0(0.1, 0.2, 0.3);
	Vector3d vertex9(10.1, 10.2, 10.4);

	EXPECT_TRUE(vertex0.isApprox(mesh->getVertex(0).position));
	EXPECT_TRUE(vertex9.isApprox(mesh->getVertex(9).position));
	EXPECT_NEAR(mesh->getMass(9)->getMass(), 0.022, 1e-6);

	ASSERT_EQ(mesh->getNumElements(), 72);
	ASSERT_EQ(mesh->getNodeIds(0).size(), 3);
	std::vector<size_t> nodeIds = { 0, 10, 1 };
	EXPECT_TRUE(std::equal(std::begin(nodeIds), std::end(nodeIds),
		std::begin(mesh->getNodeIds(0))));

	ASSERT_EQ(227u, mesh->getNumSprings());
	std::array<size_t, 2> spring0 = { 0, 1 };
	std::array<size_t, 2> spring2 = { 2, 3 };
	EXPECT_TRUE(std::equal(std::begin(spring0), std::end(spring0),
		std::begin(mesh->getSpring(0)->getNodeIds())));
	EXPECT_TRUE(std::equal(std::begin(spring2), std::end(spring2),
		std::begin(mesh->getSpring(2)->getNodeIds())));

	// Boundary conditions
	ASSERT_EQ(2u, mesh->getBoundaryConditions().size());
	ASSERT_EQ(49, mesh->getBoundaryConditions()[1]);
}

}; // Physics
}; // SurgSim

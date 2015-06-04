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

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/Fem3DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"

using SurgSim::Math::Vector3d;
using SurgSim::DataStructures::PlyReader;

namespace
{
std::string findFile(const std::string& filename)
{
	const SurgSim::Framework::ApplicationData data("config.txt");

	return data.findFile(filename);
}
}

namespace SurgSim
{
namespace Physics
{

TEST(Fem3DRepresentationReaderTests, TetrahedronMeshDelegateTest)
{
	auto fem = std::make_shared<Fem3DRepresentation>("Representation");

	std::string path = findFile("PlyReaderTests/Tetrahedron.ply");
	ASSERT_TRUE(!path.empty());
	PlyReader reader(path);
	auto delegate = std::make_shared<Fem3DPlyReaderDelegate>(fem);

	ASSERT_TRUE(reader.parseWithDelegate(delegate));

	// Vertices
	ASSERT_EQ(3u, fem->getNumDofPerNode());
	ASSERT_EQ(3u * 26u, fem->getNumDof());

	Vector3d vertex0(1.0, 1.0, -1.0);
	Vector3d vertex25(-1.0, -1.0, 1.0);

	EXPECT_TRUE(vertex0.isApprox(fem->getInitialState()->getPosition(0)));
	EXPECT_TRUE(vertex25.isApprox(fem->getInitialState()->getPosition(25)));

	// Tetrahedrons
	ASSERT_EQ(12u, fem->getNumFemElements());

	std::array<size_t, 4> tetrahedron0 = {0, 1, 2, 3};
	std::array<size_t, 4> tetrahedron2 = {10, 25, 11, 9};

	EXPECT_TRUE(std::equal(std::begin(tetrahedron0), std::end(tetrahedron0),
						   std::begin(fem->getFemElement(0)->getNodeIds())));
	EXPECT_TRUE(std::equal(std::begin(tetrahedron2), std::end(tetrahedron2),
						   std::begin(fem->getFemElement(11)->getNodeIds())));

	// Boundary conditions
	ASSERT_EQ(24u, fem->getInitialState()->getNumBoundaryConditions());

	// Boundary condition 0 is on node 8
	size_t boundaryNode0 = 8;
	size_t boundaryNode7 = 11;

	EXPECT_EQ(3 * boundaryNode0, fem->getInitialState()->getBoundaryConditions().at(0));
	EXPECT_EQ(3 * boundaryNode0 + 1, fem->getInitialState()->getBoundaryConditions().at(1));
	EXPECT_EQ(3 * boundaryNode0 + 2, fem->getInitialState()->getBoundaryConditions().at(2));
	EXPECT_EQ(3 * boundaryNode7, fem->getInitialState()->getBoundaryConditions().at(21));
	EXPECT_EQ(3 * boundaryNode7 + 1, fem->getInitialState()->getBoundaryConditions().at(22));
	EXPECT_EQ(3 * boundaryNode7 + 2, fem->getInitialState()->getBoundaryConditions().at(23));
	// Material
	auto fem2 = fem->getFemElement(2);
	EXPECT_DOUBLE_EQ(0.1432, fem2->getMassDensity());
	EXPECT_DOUBLE_EQ(0.224, fem2->getPoissonRatio());
	EXPECT_DOUBLE_EQ(0.472, fem2->getYoungModulus());

	auto fem8 = fem->getFemElement(8);
	EXPECT_DOUBLE_EQ(0.1432, fem8->getMassDensity());
	EXPECT_DOUBLE_EQ(0.224, fem8->getPoissonRatio());
	EXPECT_DOUBLE_EQ(0.472, fem8->getYoungModulus());
}

TEST(Fem3DRepresentationReaderTests, Fem3DCubePlyReadTest)
{
	auto fem = std::make_shared<Fem3DRepresentation>("Representation");

	std::string path = findFile("PlyReaderTests/Fem3DCube.ply");
	ASSERT_TRUE(!path.empty());
	PlyReader reader(path);
	auto delegate = std::make_shared<Fem3DPlyReaderDelegate>(fem);

	ASSERT_TRUE(reader.parseWithDelegate(delegate));

	// Vertices
	ASSERT_EQ(3u, fem->getNumDofPerNode());
	ASSERT_EQ(3u * 10u, fem->getNumDof());

	Vector3d vertex0(1.0, 1.0, 1.0);
	Vector3d vertex5(2.0, 2.0, 2.0);

	EXPECT_TRUE(vertex0.isApprox(fem->getInitialState()->getPosition(0)));
	EXPECT_TRUE(vertex5.isApprox(fem->getInitialState()->getPosition(5)));

	// Cubes
	ASSERT_EQ(3u, fem->getNumFemElements());

	std::array<size_t, 8> cube0 = {0, 1, 2, 3, 4, 5, 6, 7};
	std::array<size_t, 8> cube2 = {3, 4, 5, 0, 2, 6, 8, 7};

	EXPECT_TRUE(std::equal(std::begin(cube0), std::end(cube0), std::begin(fem->getFemElement(0)->getNodeIds())));
	EXPECT_TRUE(std::equal(std::begin(cube2), std::end(cube2), std::begin(fem->getFemElement(2)->getNodeIds())));

	// Boundary conditions
	ASSERT_EQ(3u* 2u, fem->getInitialState()->getNumBoundaryConditions());

	// Boundary condition 0 is on node 9
	size_t boundaryNode0 = 9;
	size_t boundaryNode1 = 5;

	EXPECT_EQ(3 * boundaryNode0, fem->getInitialState()->getBoundaryConditions().at(0));
	EXPECT_EQ(3 * boundaryNode0 + 1, fem->getInitialState()->getBoundaryConditions().at(1));
	EXPECT_EQ(3 * boundaryNode0 + 2, fem->getInitialState()->getBoundaryConditions().at(2));
	EXPECT_EQ(3 * boundaryNode1, fem->getInitialState()->getBoundaryConditions().at(3));
	EXPECT_EQ(3 * boundaryNode1 + 1, fem->getInitialState()->getBoundaryConditions().at(4));
	EXPECT_EQ(3 * boundaryNode1 + 2, fem->getInitialState()->getBoundaryConditions().at(5));

	// Material
	auto fem2 = fem->getFemElement(2);
	EXPECT_DOUBLE_EQ(0.2, fem2->getMassDensity());
	EXPECT_DOUBLE_EQ(0.3, fem2->getPoissonRatio());
	EXPECT_DOUBLE_EQ(0.4, fem2->getYoungModulus());

	auto fem1 = fem->getFemElement(1);
	EXPECT_DOUBLE_EQ(0.2, fem1->getMassDensity());
	EXPECT_DOUBLE_EQ(0.3, fem1->getPoissonRatio());
	EXPECT_DOUBLE_EQ(0.4, fem1->getYoungModulus());
}

} // Physics
} // SurgSim

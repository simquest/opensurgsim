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
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"

using SurgSim::Math::Vector3d;
using SurgSim::DataStructures::PlyReader;

namespace
{
std::string findFile(std::string filename)
{
	std::vector<std::string> paths;
	paths.push_back("Data/PlyReaderTests");
	SurgSim::Framework::ApplicationData data(paths);

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

	PlyReader reader(findFile("Tetrahedron.ply"));
	auto delegate = std::make_shared<Fem3DRepresentationPlyReaderDelegate>(fem);

	ASSERT_TRUE(reader.setDelegate(delegate));
	ASSERT_NO_THROW(reader.parseFile());

	// Vertices
	ASSERT_EQ(3u, fem->getNumDofPerNode());
	ASSERT_EQ(3u * 26u, fem->getNumDof());

	Vector3d vertex0(1.0, 1.0, -1.0);
	Vector3d vertex25(-1.0, -1.0, 1.0);

	EXPECT_TRUE(vertex0.isApprox(fem->getInitialState()->getPosition(0)));
	EXPECT_TRUE(vertex25.isApprox(fem->getInitialState()->getPosition(25)));

	// Tetrahedrons
	ASSERT_EQ(12u, fem->getNumFemElements());

	std::array<unsigned int, 4> tetrahedron0 = {0, 1, 2, 3};
	std::array<unsigned int, 4> tetrahedron11 = {10, 25, 11, 9};

	EXPECT_TRUE(std::equal(
					std::begin(tetrahedron0), std::end(tetrahedron0),
					std::begin(fem->getFemElement(0)->getNodeIds())));
	EXPECT_TRUE(std::equal(
					std::begin(tetrahedron11), std::end(tetrahedron11),
					std::begin(fem->getFemElement(11)->getNodeIds())));

	// Boundary conditions
	ASSERT_EQ(24u, fem->getInitialState()->getNumBoundaryConditions());

	// Boundary condition 0 is on node 8, this
	unsigned int boundaryNode0 = 8;
	unsigned int boundaryNode7 = 11;

	EXPECT_EQ(3 * boundaryNode0, fem->getInitialState()->getBoundaryConditions().at(0));
	EXPECT_EQ(3 * boundaryNode0 + 1, fem->getInitialState()->getBoundaryConditions().at(1));
	EXPECT_EQ(3 * boundaryNode0 + 2, fem->getInitialState()->getBoundaryConditions().at(2));
	EXPECT_EQ(3 * boundaryNode7, fem->getInitialState()->getBoundaryConditions().at(21));
	EXPECT_EQ(3 * boundaryNode7 + 1, fem->getInitialState()->getBoundaryConditions().at(22));
	EXPECT_EQ(3 * boundaryNode7 + 2, fem->getInitialState()->getBoundaryConditions().at(23));
	// Material
	auto fem2 = fem->getFemElement(2);
	EXPECT_EQ(0.1432, fem2->getMassDensity());
	EXPECT_EQ(0.224, fem2->getPoissonRatio());
	EXPECT_EQ(0.472, fem2->getYoungModulus());

	auto fem8 = fem->getFemElement(8);
	EXPECT_EQ(0.1432, fem2->getMassDensity());
	EXPECT_EQ(0.224, fem2->getPoissonRatio());
	EXPECT_EQ(0.472, fem2->getYoungModulus());
}

}
}


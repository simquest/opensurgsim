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
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"

using SurgSim::Math::Vector3d;
using SurgSim::DataStructures::PlyReader;

namespace {
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
	PlyReader reader(findFile("Tetrahedron.ply"));
	auto delegate = std::make_shared<Fem3DRepresentationPlyReaderDelegate>();

	ASSERT_TRUE(reader.setDelegate(delegate));
	ASSERT_NO_THROW(reader.parseFile());

	auto fem = delegate->getFem();

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

	EXPECT_TRUE(
		std::equal(std::begin(tetrahedron0), std::end(tetrahedron0), std::begin(fem->getFemElement(0)->getNodeIds())));
	EXPECT_TRUE(std::equal(
		std::begin(tetrahedron11), std::end(tetrahedron11), std::begin(fem->getFemElement(11)->getNodeIds())));

	// Boundary conditions
	ASSERT_EQ(8u, fem->getInitialState()->getNumBoundaryConditions());

	unsigned int boundaryCondition0 = 8;
	unsigned int boundaryConditoin7 = 11;

	EXPECT_EQ(boundaryCondition0, fem->getInitialState()->getBoundaryConditions().at(0));
	EXPECT_EQ(boundaryConditoin7, fem->getInitialState()->getBoundaryConditions().at(7));
}

}
}


// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem.h"
#include "SurgSim/Physics/Fem3DPlyReaderDelegate.h"

using SurgSim::Math::Vector3d;
using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

TEST(Fem3DRepresentationReaderTests, TetrahedronMeshDelegateTest)
{
	auto fem = std::make_shared<Fem3D>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	fem->load("PlyReaderTests/Tetrahedron.ply");

	// Vertices
	Vector3d vertex0(1.0, 1.0, -1.0);
	Vector3d vertex25(-1.0, -1.0, 1.0);

	EXPECT_TRUE(vertex0.isApprox(fem->getVertex(0).position));
	EXPECT_TRUE(vertex25.isApprox(fem->getVertex(25).position));

	// Tetrahedrons
	ASSERT_EQ(12u, fem->getNumElements());

	std::array<size_t, 4> tetrahedron0 = {0, 1, 2, 3};
	std::array<size_t, 4> tetrahedron2 = {10, 25, 11, 9};

	EXPECT_TRUE(std::equal(std::begin(tetrahedron0), std::end(tetrahedron0),
						   std::begin(fem->getElement(0)->nodeIds)));
	EXPECT_TRUE(std::equal(std::begin(tetrahedron2), std::end(tetrahedron2),
						   std::begin(fem->getElement(11)->nodeIds)));

	// Boundary conditions
	ASSERT_EQ(8u, fem->getBoundaryConditions().size());

	EXPECT_EQ(8, fem->getBoundaryCondition(0));
	EXPECT_EQ(5, fem->getBoundaryCondition(1));
	EXPECT_EQ(3, fem->getBoundaryCondition(2));
	EXPECT_EQ(2, fem->getBoundaryCondition(3));
	EXPECT_EQ(7, fem->getBoundaryCondition(4));
	EXPECT_EQ(1, fem->getBoundaryCondition(5));
	EXPECT_EQ(6, fem->getBoundaryCondition(6));
	EXPECT_EQ(11, fem->getBoundaryCondition(7));

	// Material
	auto fem2 = fem->getElement(2);
	EXPECT_DOUBLE_EQ(0.1432, fem2->massDensity);
	EXPECT_DOUBLE_EQ(0.224, fem2->poissonRatio);
	EXPECT_DOUBLE_EQ(0.472, fem2->youngModulus);

	auto fem8 = fem->getElement(8);
	EXPECT_DOUBLE_EQ(0.1432, fem8->massDensity);
	EXPECT_DOUBLE_EQ(0.224, fem8->poissonRatio);
	EXPECT_DOUBLE_EQ(0.472, fem8->youngModulus);
}

TEST(Fem3DRepresentationReaderTests, CubeMeshDelegateTest)
{
	auto fem = std::make_shared<Fem3D>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	fem->load("PlyReaderTests/Fem3DCube.ply");

	// Vertices
	Vector3d vertex0(1.0, 1.0, 1.0);
	Vector3d vertex5(2.0, 2.0, 2.0);

	EXPECT_TRUE(vertex0.isApprox(fem->getVertex(0).position));
	EXPECT_TRUE(vertex5.isApprox(fem->getVertex(5).position));

	// Cubes
	ASSERT_EQ(3u, fem->getNumElements());

	std::array<size_t, 8> cube0 = {0, 1, 2, 3, 4, 5, 6, 7};
	std::array<size_t, 8> cube2 = {3, 4, 5, 0, 2, 6, 8, 7};

	EXPECT_TRUE(std::equal(std::begin(cube0), std::end(cube0), std::begin(fem->getElement(0)->nodeIds)));
	EXPECT_TRUE(std::equal(std::begin(cube2), std::end(cube2), std::begin(fem->getElement(2)->nodeIds)));

	// Boundary conditions
	ASSERT_EQ(2u, fem->getBoundaryConditions().size());

	EXPECT_EQ(9, fem->getBoundaryCondition(0));
	EXPECT_EQ(5, fem->getBoundaryCondition(1));

	// Material
	auto fem2 = fem->getElement(2);
	EXPECT_DOUBLE_EQ(0.2, fem2->massDensity);
	EXPECT_DOUBLE_EQ(0.3, fem2->poissonRatio);
	EXPECT_DOUBLE_EQ(0.4, fem2->youngModulus);

	auto fem1 = fem->getElement(1);
	EXPECT_DOUBLE_EQ(0.2, fem1->massDensity);
	EXPECT_DOUBLE_EQ(0.3, fem1->poissonRatio);
	EXPECT_DOUBLE_EQ(0.4, fem1->youngModulus);
}

TEST(Fem3DRepresentationReaderTests, WrongPlyWithRotationDof)
{
	auto fem = std::make_shared<Fem3D>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	ASSERT_NO_THROW(fem->load("PlyReaderTests/Wrong3DFileWithRotationData.ply"));
}

TEST(Fem3DRepresentationReaderTests, PerElementMaterial)
{
	auto fem = std::make_shared<Fem3D>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	fem->load("PlyReaderTests/Fem3DCubeMaterial.ply");

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

TEST(Fem3DRepresentationReaderTests, NoMaterials)
{
	auto fem = std::make_shared<Fem3D>();
	auto runtime = std::make_shared<Framework::Runtime>("config.txt");

	ASSERT_NO_THROW(fem->load("PlyReaderTests/Fem3DCubeNoMaterial.ply"));

	for (auto element : fem->getElements())
	{
		EXPECT_DOUBLE_EQ(0.0, element->massDensity);
		EXPECT_DOUBLE_EQ(0.0, element->poissonRatio);
		EXPECT_DOUBLE_EQ(0.0, element->youngModulus);
	}
}

} // Physics
} // SurgSim

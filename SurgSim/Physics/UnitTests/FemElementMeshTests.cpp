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

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Physics/FemElementMesh.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

TEST(FemElementMeshReaderTests, DelegateTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	std::string filePath = runtime->getApplicationData()->findFile("FemElementMeshTests/Fem1D.ply");

	auto mesh1d = std::make_shared<FemElement1DMesh>();
	mesh1d->load(filePath);

	EXPECT_EQ(4, mesh1d->getNumElements());
	EXPECT_EQ(3, mesh1d->getBoundaryConditions().size());
	for (auto element : mesh1d->getFemElements())
	{
		EXPECT_DOUBLE_EQ(0.11, element->radius);
		EXPECT_DOUBLE_EQ(0.21, element->massDensity);
		EXPECT_DOUBLE_EQ(0.31, element->poissonRatio);
		EXPECT_DOUBLE_EQ(0.41, element->youngModulus);
	}

	filePath = runtime->getApplicationData()->findFile("FemElementMeshTests/Fem2D.ply");

	auto mesh2d = std::make_shared<FemElement2DMesh>();
	mesh2d->load(filePath);

	EXPECT_EQ(3, mesh2d->getNumElements());
	EXPECT_EQ(2, mesh2d->getBoundaryConditions().size());
	for (auto element : mesh2d->getFemElements())
	{
		EXPECT_DOUBLE_EQ(0.1, element->thickness);
		EXPECT_DOUBLE_EQ(0.2, element->massDensity);
		EXPECT_DOUBLE_EQ(0.3, element->poissonRatio);
		EXPECT_DOUBLE_EQ(0.4, element->youngModulus);
	}
}

} // namespace Physics
} // namespace SurgSim

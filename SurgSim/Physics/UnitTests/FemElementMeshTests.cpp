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

#include "SurgSim/Physics/FemElementMesh.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

TEST(FemElement1DMeshReaderTests, DelegateTest)
{
	FemElement1DMesh mesh;
	mesh.load("FemElementMeshTests/Fem1D.ply");

	EXPECT_EQ(4, mesh.getNumElements());
	EXPECT_EQ(0.11, mesh.getRadius());
	EXPECT_EQ(0.21, mesh.getMassDensity());
	EXPECT_EQ(0.31, mesh.getPoissonRatio());
	EXPECT_EQ(0.41, mesh.getYoungModulus());
}

} // namespace Physics
} // namespace SurgSim

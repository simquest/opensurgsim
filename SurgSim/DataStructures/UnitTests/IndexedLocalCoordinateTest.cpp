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

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace DataStructures
{

TEST(IndexedLocalCoordinateTest, IndexedLocalCoordinate)
{
	using SurgSim::Math::Vector4d;

	EXPECT_NO_THROW({
		IndexedLocalCoordinate coord;
	});

	EXPECT_NO_THROW({
		IndexedLocalCoordinate coord(6u, Vector4d(0.25, 0.55, 0.73, 0.11));
	});

	{
		IndexedLocalCoordinate coord(6u, Vector4d(0.25, 0.55, 0.73, 0.11));
		EXPECT_EQ(6u, coord.elementId);
		EXPECT_TRUE(Vector4d(0.25, 0.55, 0.73, 0.11).isApprox(coord.barycentricCoordinate));
	}

	{
		IndexedLocalCoordinate coord;
		coord.elementId = 12u;
		coord.barycentricCoordinate = Vector4d(0.33, 0.1, 0.05, 0.99);
		EXPECT_EQ(12u, coord.elementId);
		EXPECT_TRUE(Vector4d(0.33, 0.1, 0.05, 0.99).isApprox(coord.barycentricCoordinate));
	}

	{
		IndexedLocalCoordinate coord0;
		coord0.elementId = 0u;
		SurgSim::Math::Vector cubeNodes(8);
		cubeNodes << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

		coord0.barycentricCoordinate = cubeNodes;
		EXPECT_EQ(0u, coord0.elementId);
		EXPECT_TRUE(cubeNodes.isApprox(coord0.barycentricCoordinate));
	}
}

} // namespace Collision
} // namespace SurgSim
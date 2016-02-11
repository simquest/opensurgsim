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
		EXPECT_EQ(6u, coord.index);
		EXPECT_TRUE(Vector4d(0.25, 0.55, 0.73, 0.11).isApprox(coord.coordinate));
	}

	{
		IndexedLocalCoordinate coord;
		coord.index = 12u;
		coord.coordinate = Vector4d(0.33, 0.1, 0.05, 0.99);
		EXPECT_EQ(12u, coord.index);
		EXPECT_TRUE(Vector4d(0.33, 0.1, 0.05, 0.99).isApprox(coord.coordinate));
	}

	{
		IndexedLocalCoordinate coord0;
		coord0.index = 0u;
		SurgSim::Math::Vector cubeNodes(8);
		cubeNodes << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

		coord0.coordinate = cubeNodes;
		EXPECT_EQ(0u, coord0.index);
		EXPECT_TRUE(cubeNodes.isApprox(coord0.coordinate));
	}
}

TEST(IndexedLocalCoordinateTest, IsApprox)
{
	using SurgSim::Math::Vector3d;
	double epsilon = 1e-12;

	IndexedLocalCoordinate null6(6u, Vector3d::Zero());
	IndexedLocalCoordinate almostNull6(6u, Vector3d(0.0, 0.0, epsilon / 2.0));
	IndexedLocalCoordinate null5(5u, Vector3d::Zero());
	IndexedLocalCoordinate almostNull5(5u, Vector3d(0.0, 0.0, epsilon / 2.0));

	EXPECT_TRUE(null6.isApprox(null6, epsilon));
	EXPECT_TRUE(null6.isApprox(almostNull6, epsilon));
	EXPECT_FALSE(null6.isApprox(null5, epsilon));
	EXPECT_FALSE(null6.isApprox(almostNull5, epsilon));

	IndexedLocalCoordinate one6(6u, Vector3d::Ones());
	IndexedLocalCoordinate almostOne6(6u, Vector3d::Ones() + Vector3d(0.0, 0.0, epsilon / 2.0));
	IndexedLocalCoordinate one5(5u, Vector3d::Ones());
	IndexedLocalCoordinate two6(6u, Vector3d::Constant(2.0));
	IndexedLocalCoordinate three7(7u, Vector3d::Constant(3.0));

	EXPECT_TRUE(one6.isApprox(one6, epsilon));
	EXPECT_TRUE(one6.isApprox(almostOne6, epsilon));
	EXPECT_FALSE(one6.isApprox(one5, epsilon));
	EXPECT_FALSE(one6.isApprox(two6, epsilon));
	EXPECT_FALSE(one6.isApprox(three7, epsilon));
}

} // namespace Collision
} // namespace SurgSim
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

#include "SurgSim/Math/Aabb.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Math
{

TEST(AabbTests, DoIntersectTest)
{
	Aabbd center(Vector3d(-1.0, -1.0, -1.0), Vector3d(1.0, 1.0, 1.0));

	// Move the sampler around in all directions only when one of the factors is greater than 2
	// Should the intersection fail

	for (double i = -3.0; i <= 3.0; i += 1)
	{
		for (double j = -3.0; j <= 3.0; j += 1)
		{
			for (double k = -3.0; k <= 3.0; k += 1)
			{
				Vector3d translation(i, j, k);
				/// Whenever all of the coordinates are <= 2.0 the result should be true
				bool expected = (translation.cwiseAbs().array().abs() <= Vector3d(2.0, 2.0, 2.0).array()).all();
				Aabbd sampler = Aabbd(center).translate(translation);

				EXPECT_EQ(expected, doAabbIntersect(center, sampler)) << "Error For " << translation.transpose();
				EXPECT_EQ(expected, doAabbIntersect(sampler, center)) << "Error For " << translation.transpose();

				EXPECT_EQ(expected, doAabbIntersect(center, sampler, 0.0)) << "Error For " << translation.transpose();
				EXPECT_EQ(expected, doAabbIntersect(sampler, center, 0.0)) << "Error For " << translation.transpose();

				double tolerance = 0.01;
				expected = (translation.cwiseAbs().array().abs() <=
							Vector3d(2.0 + tolerance, 2.0 + tolerance, 2.0 + tolerance).array()).all();

				EXPECT_EQ(expected, doAabbIntersect(center, sampler, tolerance))
						<< "Error For " << tolerance << " " << translation.transpose();
				EXPECT_EQ(expected, doAabbIntersect(sampler, center, tolerance))
						<< "Error For " << tolerance << " " << translation.transpose();
			}
		}
	}
}

TEST(AabbdTests, makeAabb)
{
	Vector3d one(0.0, 0.0, 0.0);
	Vector3d two(-1.0, -1.0, -1.0);
	Vector3d three(1.0, 1.0, 1.0);
	Aabbd aabb(makeAabb(one, two, three));
	Aabbd expected(Vector3d(-1.0, -1.0, -1.0), Vector3d(1.0, 1.0, 1.0));
	EXPECT_TRUE(expected.isApprox(aabb));

	Vector3d four(2.0, 2.0, 2.0);
	Aabbd aabb2(makeAabb(one, two, three, four));
	Aabbd expected2(Vector3d(-1.0, -1.0, -1.0), Vector3d(2.0, 2.0, 2.0));
	EXPECT_TRUE(expected2.isApprox(aabb2));
}

}
}


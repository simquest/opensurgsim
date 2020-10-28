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

#include "SurgSim/DataStructures/AabbTreeData.h"

#include "SurgSim/Math/Aabb.h"
#include "SurgSim/Math/Vector.h"

#include "SurgSim/Testing/MathUtilities.h"

using SurgSim::Math::Aabbd;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace DataStructures
{

TEST(AabbTreeDataTests, InitTest)
{
	ASSERT_NO_THROW(AabbTreeData data;);

	AabbTreeData data;
	EXPECT_TRUE(data.isEmpty());
	EXPECT_EQ(0u, data.getSize());
}

TEST(AabbTreeDataTests, EqualityTests)
{
	Aabbd a(Vector3d(-1.0, -1.0, -1.0), Vector3d(1.0, 1.0, 1.0));
	Aabbd b(Vector3d(-2.0, -2.0, -2.0), Vector3d(0.0, 0.0, 0.0));

	AabbTreeData empty;
	AabbTreeData containsA;
	AabbTreeData containsAnotherA;
	AabbTreeData containsB;
	AabbTreeData containsAB;
	AabbTreeData containsBA;

	containsA.add(a, 0);
	containsAnotherA.add(a, 0);
	containsB.add(b, 0);

	containsAB.add(a, 0);
	containsAB.add(b, 0);

	containsBA.add(b, 0);
	containsBA.add(a, 0);

	EXPECT_EQ(empty, empty);
	EXPECT_NE(empty, containsA);
	EXPECT_NE(empty, containsAB);
	EXPECT_NE(containsA, empty);
	EXPECT_NE(containsBA, empty);

	EXPECT_EQ(containsA, containsA);
	EXPECT_EQ(containsA, containsAnotherA);

	EXPECT_NE(containsA, containsB);
	EXPECT_NE(containsA, containsAB);

	EXPECT_EQ(containsAB, containsAB);
	EXPECT_EQ(containsAB, containsBA);
	EXPECT_EQ(containsBA, containsAB);
}

TEST(AabbTreeDataTests, AddTest)
{
	Aabbd a(Vector3d(-1.0, -1.0, -1.0), Vector3d(1.0, 1.0, 1.0));
	Aabbd b(Vector3d(-2.0, -2.0, -2.0), Vector3d(0.0, 0.0, 0.0));
	Aabbd c = a.merged(b);

	AabbTreeData data;

	EXPECT_NO_THROW(data.add(a, 0));
	EXPECT_TRUE(a.isApprox(data.getAabb()));
	EXPECT_EQ(1u, data.getSize());
	data.add(b, 1);

	EXPECT_EQ(2u, data.getSize());
	EXPECT_TRUE(c.isApprox(data.getAabb()));
}

TEST(AabbTreeDataTests, SimpleSplitTest)
{
	AabbTreeData data;

	// add 11 elements
	for (int i = 0; i <= 10; ++i)
	{
		data.add(Aabbd(Vector3d(i, -1.0, -1.0), Vector3d(i + 2, 1.0, 1.0)), i);
	}

	// The original box, encompasses all the members
	Aabbd original(Vector3d(0.0, -1.0, -1.0), Vector3d(12.0, 1.0, 1.0));

	// The left hand side box, this is what is left in the original data item
	// all the items with center.x < (12.0 - 0.0) / 2.0, 5 items
	Aabbd expectedLeft(Vector3d(0.0, -1.0, -1.0), Vector3d(6.0, 1.0, 1.0));

	// The left hand side box, this is what moved to the new data item
	// all the items with center.x >= (12.0 - 0.0) / 2.0, 6 items
	Aabbd expectedRight(Vector3d(5.0, -1.0, -1.0), Vector3d(12.0, 1.0, 1.0));

	EXPECT_TRUE(original.isApprox(data.getAabb())) << original << ", " << data.getAabb();
	std::shared_ptr<AabbTreeData> ptr = data.takeLargerElements();

	EXPECT_EQ(5u, data.getSize());
	EXPECT_TRUE(expectedLeft.isApprox(data.getAabb())) << expectedLeft << ", " << data.getAabb();

	EXPECT_EQ(6u, ptr->getSize());
	EXPECT_TRUE(expectedRight.isApprox(ptr->getAabb())) << expectedRight << ", " << ptr->getAabb();
}

TEST(AabbTreeDataTests, IntersectionTest)
{
	Aabbd a(Vector3d(-1.0, -1.0, -1.0), Vector3d(1.0, 1.0, 1.0));
	Aabbd b(Vector3d(-2.0, -2.0, -2.0), Vector3d(0.0, 0.0, 0.0));

	AabbTreeData data;
	data.add(Aabbd(Vector3d(-1.0, 0.0, 0.0), Vector3d(-1.0, 0.0, 0.0)), 0);
	data.add(Aabbd(Vector3d(-2.0, 0.0, 0.0), Vector3d(-2.0, 0.0, 0.0)), 1);
	data.add(Aabbd(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)), 2);
	data.add(Aabbd(Vector3d(1.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0)), 3);


	Aabbd noIntersections(Vector3d(-0.5, -2.0, -2.0), Vector3d(0.5, -1.0, -1.0));
	Aabbd oneIntersection(Vector3d(-0.5, 0.0, 0.0), Vector3d(0.5, 0.0, 0.0));
	Aabbd twoIntersections(Vector3d(-0.5, 0.0, 0.0), Vector3d(1.5, 0.0, 0.0));
	Aabbd threeIntersections(Vector3d(-1.5, 0.0, 0.0), Vector3d(1.5, 0.0, 0.0));
	Aabbd fourIntersections(Vector3d(-2.5, 0.0, 0.0), Vector3d(2.5, 0.0, 0.0));


	std::vector<size_t> results;
	EXPECT_FALSE(data.hasIntersections(noIntersections));
	data.getIntersections(noIntersections, &results);
	EXPECT_EQ(0u, results.size());
	results.size();

	results.clear();
	EXPECT_TRUE(data.hasIntersections(oneIntersection));
	data.getIntersections(oneIntersection, &results);
	EXPECT_EQ(1u, results.size());
	EXPECT_EQ(2u, results.front());

	results.clear();
	EXPECT_TRUE(data.hasIntersections(twoIntersections));
	data.getIntersections(twoIntersections, &results);
	EXPECT_EQ(2u, results.size());

	results.clear();
	EXPECT_TRUE(data.hasIntersections(threeIntersections));
	data.getIntersections(threeIntersections, &results);
	EXPECT_EQ(3u, results.size());

	results.clear();
	EXPECT_TRUE(data.hasIntersections(fourIntersections));
	data.getIntersections(fourIntersections, &results);
	EXPECT_EQ(4u, results.size());
}




}
}


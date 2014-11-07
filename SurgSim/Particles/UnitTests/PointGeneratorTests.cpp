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

#include <memory>
#include <vector>

#include "SurgSim/Math/Aabb.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/RandomBoxPointGenerator.h"
#include "SurgSim/Particles/RandomSpherePointGenerator.h"

using SurgSim::Math::BoxShape;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;
using SurgSim::Particles::RandomBoxPointGenerator;
using SurgSim::Particles::RandomSpherePointGenerator;

TEST(PointGeneratorTest, ConstructorTest)
{
	ASSERT_NO_THROW(RandomBoxPointGenerator());
	ASSERT_NO_THROW(RandomSpherePointGenerator());
}

TEST(PointGeneratorTest, BoxPointGeneratorTest)
{
	auto boxShape = std::make_shared<BoxShape>(2.0, 4.0, 6.0);
	auto aabb = SurgSim::Math::Aabbd(Vector3d(-1.0, -2.0, -3.0), Vector3d(1.0, 2.0, 3.0));
	auto boxPointGenerator = std::make_shared<RandomBoxPointGenerator>();

	auto pointInsideBox = boxPointGenerator->pointInShape(boxShape);
	bool isInOrOnBox = ((pointInsideBox - aabb.min()).array() > -SurgSim::Math::Geometry::ScalarEpsilon).all() && 
					   ((pointInsideBox - aabb.max()).array() < SurgSim::Math::Geometry::ScalarEpsilon).all();
	EXPECT_TRUE(isInOrOnBox); // The point is located in the aabb
	size_t intersection = aabb.min().cwiseEqual(pointInsideBox).count() + 
						  aabb.max().cwiseEqual(pointInsideBox).count();
	EXPECT_EQ(intersection, 0u); // The point is NOT on the surface of the aabb

	auto pointOnBox = boxPointGenerator->pointOnShape(boxShape);
	isInOrOnBox = ((pointOnBox - aabb.min()).array() >= -SurgSim::Math::Geometry::ScalarEpsilon).all() && 
				  ((pointOnBox - aabb.max()).array() <= SurgSim::Math::Geometry::ScalarEpsilon).all();
	EXPECT_TRUE(isInOrOnBox); // The point is located in the aabb
	intersection = aabb.min().cwiseEqual(pointOnBox).count() + 
				   aabb.max().cwiseEqual(pointOnBox).count();
	EXPECT_GT(intersection, 0u); // The point IS on the surface of the aabb
}

TEST(PointGeneratorTest, SpherePointGeneratorTest)
{
	auto sphereShape = std::make_shared<SphereShape>(2.0);
	auto spherePointGenerator = std::make_shared<RandomSpherePointGenerator>();

	auto pointInsideSphere = spherePointGenerator->pointInShape(sphereShape);
	EXPECT_LT(pointInsideSphere.norm(), sphereShape->getRadius());

	auto pointOnSphere = spherePointGenerator->pointOnShape(sphereShape);
	EXPECT_NEAR(sphereShape->getRadius(), pointOnSphere.norm(), SurgSim::Math::Geometry::DistanceEpsilon);
}

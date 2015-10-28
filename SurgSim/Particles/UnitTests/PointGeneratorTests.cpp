// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Math/Aabb.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/RandomBoxPointGenerator.h"
#include "SurgSim/Particles/RandomMeshPointGenerator.h"
#include "SurgSim/Particles/RandomSpherePointGenerator.h"

using SurgSim::Math::BoxShape;
using SurgSim::Math::Geometry::DistanceEpsilon;
using SurgSim::Math::MeshShape;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;
using SurgSim::Particles::RandomBoxPointGenerator;
using SurgSim::Particles::RandomMeshPointGenerator;
using SurgSim::Particles::RandomSpherePointGenerator;


TEST(PointGeneratorTest, ConstructorTest)
{
	ASSERT_NO_THROW(RandomBoxPointGenerator());
	ASSERT_NO_THROW(RandomMeshPointGenerator());
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
	EXPECT_TRUE(isInOrOnBox) << "Point should be in or on the box " << pointInsideBox;
	bool intersection = ((aabb.min() - pointInsideBox).array().abs() < SurgSim::Math::Geometry::ScalarEpsilon).any() ||
						((aabb.max() - pointInsideBox).array().abs() < SurgSim::Math::Geometry::ScalarEpsilon).any();
	EXPECT_FALSE(intersection) << "Point should not be on surface of box " << pointInsideBox;

	auto pointOnBox = boxPointGenerator->pointOnShape(boxShape);
	isInOrOnBox = ((pointOnBox - aabb.min()).array() >= -SurgSim::Math::Geometry::ScalarEpsilon).all() &&
				  ((pointOnBox - aabb.max()).array() <= SurgSim::Math::Geometry::ScalarEpsilon).all();
	EXPECT_TRUE(isInOrOnBox) << "Point should be in or on the box " << pointOnBox;
	bool intersection2 = ((aabb.min() - pointOnBox).array().abs() < SurgSim::Math::Geometry::ScalarEpsilon).any() ||
						 ((aabb.max() - pointOnBox).array().abs() < SurgSim::Math::Geometry::ScalarEpsilon).any();
	EXPECT_TRUE(intersection2) << "Point should be on surface of box " << pointOnBox;
}

TEST(PointGeneratorTest, MeshPointGeneratorTest)
{
	auto meshShape = std::make_shared<MeshShape>();
	std::array<size_t, 3> triangleIds;

	triangleIds[0] = meshShape->addVertex(MeshShape::VertexType(Vector3d(-1.0, 1.0, 1.0)));
	triangleIds[1] = meshShape->addVertex(MeshShape::VertexType(Vector3d(-1.0, 5.0, 1.0)));
	triangleIds[2] = meshShape->addVertex(MeshShape::VertexType(Vector3d(-1.0, 1.0, 5.0)));
	meshShape->addTriangle(MeshShape::TriangleType(triangleIds));

	triangleIds[0] = meshShape->addVertex(MeshShape::VertexType(Vector3d(1.0, 1.0, 1.0)));
	triangleIds[1] = meshShape->addVertex(MeshShape::VertexType(Vector3d(1.0, 5.0, 1.0)));
	triangleIds[2] = meshShape->addVertex(MeshShape::VertexType(Vector3d(1.0, 1.0, 5.0)));
	size_t triangleId = meshShape->addTriangle(MeshShape::TriangleType(triangleIds));
	meshShape->getTriangle(triangleId).isValid = false;

	auto meshPointGenerator = std::make_shared<RandomMeshPointGenerator>();
	auto pointOnMesh = meshPointGenerator->pointOnShape(meshShape);

	EXPECT_NEAR(-1.0, pointOnMesh[0], DistanceEpsilon)
		<< "Point is not on a triangle, or was generated with an invalid triangle.";
	EXPECT_LE(1.0, pointOnMesh[1]);
	EXPECT_LE(1.0, pointOnMesh[2]);
	EXPECT_GE(6.0, pointOnMesh[1] + pointOnMesh[2]);
}

TEST(PointGeneratorTest, SpherePointGeneratorTest)
{
	auto sphereShape = std::make_shared<SphereShape>(2.0);
	auto spherePointGenerator = std::make_shared<RandomSpherePointGenerator>();

	auto pointInsideSphere = spherePointGenerator->pointInShape(sphereShape);
	EXPECT_LT(pointInsideSphere.norm(), sphereShape->getRadius());

	auto pointOnSphere = spherePointGenerator->pointOnShape(sphereShape);
	EXPECT_NEAR(sphereShape->getRadius(), pointOnSphere.norm(), DistanceEpsilon);
}

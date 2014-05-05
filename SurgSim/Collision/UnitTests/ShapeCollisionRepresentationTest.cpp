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

#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace {
static const double dt = 0.001;
}

namespace SurgSim
{
namespace Collision
{

TEST(ShapeCollisionRepresentationTest, MeshUpdateTest)
{
	const std::string fileName = "MeshShapeData/staple_collision.ply";
	auto meshShape = std::make_shared<SurgSim::Math::MeshShape>();
	meshShape->setFileName(fileName);

	auto collisionRepresentation = std::make_shared<ShapeCollisionRepresentation>(
		"Collision", meshShape, SurgSim::Math::RigidTransform3d::Identity());
	collisionRepresentation->update(dt);

	auto originalMesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>(*meshShape->getMesh());
	auto expectedMesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>(*meshShape->getMesh());
	auto actualMesh
		= std::static_pointer_cast<SurgSim::Math::MeshShape>(collisionRepresentation->getShape())->getMesh();

	EXPECT_EQ(expectedMesh->getVertices(), actualMesh->getVertices());
	EXPECT_EQ(expectedMesh->getTriangles(), actualMesh->getTriangles());

	RigidTransform3d transform = SurgSim::Math::makeRigidTransform(
		Vector3d(4.3, 2.1, 6.5), Vector3d(-1.5, 7.5, -2.5), Vector3d(8.7, -4.7, -3.1));

	collisionRepresentation->setPose(transform);
	collisionRepresentation->update(dt);

	expectedMesh->copyWithTransform(transform, *originalMesh);

	EXPECT_EQ(expectedMesh->getVertices(), actualMesh->getVertices());
	EXPECT_EQ(expectedMesh->getTriangles(), actualMesh->getTriangles());
}

}
}

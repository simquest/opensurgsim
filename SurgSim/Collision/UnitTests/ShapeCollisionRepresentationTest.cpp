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
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace
{
static const double dt = 0.001;
}

namespace SurgSim
{
namespace Collision
{

TEST(ShapeCollisionRepresentationTest, MeshUpdateTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	const std::string fileName = "staple_collision.ply";
	auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMeshPlain>();
	ASSERT_NO_THROW(mesh->load(fileName));

	auto originalMesh = std::make_shared<SurgSim::Math::MeshShape>(*mesh);
	auto expectedMesh = std::make_shared<SurgSim::Math::MeshShape>(*mesh);

	auto collisionRepresentation = std::make_shared<ShapeCollisionRepresentation>("Collision");
	collisionRepresentation->setShape(originalMesh);
	collisionRepresentation->setLocalPose(SurgSim::Math::RigidTransform3d::Identity());
	collisionRepresentation->update(dt);

	EXPECT_EQ(collisionRepresentation->getShapeType(), originalMesh->getType());

	auto actualMesh = std::static_pointer_cast<SurgSim::Math::MeshShape>(collisionRepresentation->getPosedShape());
	EXPECT_EQ(expectedMesh->getVertices(), actualMesh->getVertices());
	EXPECT_EQ(expectedMesh->getTriangles(), actualMesh->getTriangles());

	RigidTransform3d transform = SurgSim::Math::makeRigidTransform(Vector3d(4.3, 2.1, 6.5), Vector3d(-1.5, 7.5, -2.5),
			Vector3d(8.7, -4.7, -3.1));
	collisionRepresentation->setLocalPose(transform);
	collisionRepresentation->update(dt);

	actualMesh = std::static_pointer_cast<SurgSim::Math::MeshShape>(collisionRepresentation->getPosedShape());
	expectedMesh->transform(transform);
	EXPECT_TRUE(expectedMesh->update());
	EXPECT_EQ(expectedMesh->getVertices(), actualMesh->getVertices());
	EXPECT_EQ(expectedMesh->getTriangles(), actualMesh->getTriangles());
}

TEST(ShapeCollisionRepresentationTest, SerializationTest)
{
	SurgSim::Framework::Runtime runtime("config.txt");

	const std::string fileName = "staple_collision.ply";
	std::shared_ptr<SurgSim::Math::Shape> shape = std::make_shared<SurgSim::Math::MeshShape>();
	auto meshShape = std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(shape);
	EXPECT_NO_THROW(meshShape->load(fileName));

	auto collisionRepresentation = std::make_shared<ShapeCollisionRepresentation>("Collision");
	collisionRepresentation->setValue("Shape", shape);
	RigidTransform3d pose = SurgSim::Math::makeRigidTransform(Vector3d(4.3, 2.1, 6.5),
							Vector3d(-1.5, 7.5, -2.5),
							Vector3d(8.7, -4.7, -3.1));
	collisionRepresentation->setLocalPose(pose);

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*collisionRepresentation));
	EXPECT_EQ(1u, node.size());

	YAML::Node data = node["SurgSim::Collision::ShapeCollisionRepresentation"];
	EXPECT_EQ(5u, data.size());

	std::shared_ptr<ShapeCollisionRepresentation> newShapeCollisionRepresentation;
	ASSERT_NO_THROW(newShapeCollisionRepresentation = std::dynamic_pointer_cast<ShapeCollisionRepresentation>
					(node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
	EXPECT_TRUE(pose.isApprox(newShapeCollisionRepresentation->getPose()));

	auto mesh = std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(
					newShapeCollisionRepresentation->getValue<std::shared_ptr<SurgSim::Math::Shape>>("Shape"));
	ASSERT_TRUE(nullptr != mesh);
	EXPECT_EQ(meshShape->getNumEdges(), mesh->getNumEdges());
	EXPECT_EQ(meshShape->getNumTriangles(), mesh->getNumTriangles());
	EXPECT_EQ(meshShape->getNumVertices(), mesh->getNumVertices());
}

} // namespace Collision
} // namespace SurgSim

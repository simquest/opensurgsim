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

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Framework/Runtime.h"

using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace
{
static const double dt = 0.001;
const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Physics
{

struct RigidCollisionRepresentationTest : public ::testing::Test
{
	void SetUp()
	{
		m_sphereShape = std::make_shared<SurgSim::Math::SphereShape>(1.0);
		m_rigidRepresentation = std::make_shared<RigidRepresentation>("RigidRepresentation");
		m_rigidRepresentation->setShape(m_sphereShape);
	}

	std::shared_ptr<SurgSim::Math::SphereShape> m_sphereShape;
	std::shared_ptr<SurgSim::Physics::RigidCollisionRepresentation> m_rigidCollisionRepresentation;
	std::shared_ptr<SurgSim::Physics::RigidRepresentation> m_rigidRepresentation;
};

TEST_F(RigidCollisionRepresentationTest, InitTest)
{
	EXPECT_NO_THROW(RigidCollisionRepresentation("TestRigidCollisionRepresentation"));
	EXPECT_NO_THROW(m_rigidCollisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation")
	);

	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	EXPECT_FALSE(m_rigidCollisionRepresentation->initialize(runtime)); // no physic rep
}

TEST_F(RigidCollisionRepresentationTest, SetGetRigidRepresentationTest)
{
	m_rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");
	ASSERT_NO_THROW(m_rigidCollisionRepresentation->setRigidRepresentation(m_rigidRepresentation));
	EXPECT_EQ(m_rigidRepresentation, m_rigidCollisionRepresentation->getRigidRepresentation());

	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	EXPECT_TRUE(m_rigidCollisionRepresentation->initialize(runtime));
	EXPECT_TRUE(m_rigidCollisionRepresentation->wakeUp());
}

TEST_F(RigidCollisionRepresentationTest, WakeUpTest)
{
	m_rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");
	auto physicsNoShape = std::make_shared<RigidRepresentation>("RigidRepresentation");
	m_rigidCollisionRepresentation->setRigidRepresentation(physicsNoShape);

	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	EXPECT_TRUE(m_rigidCollisionRepresentation->initialize(runtime));
	EXPECT_FALSE(m_rigidCollisionRepresentation->wakeUp());
}

TEST_F(RigidCollisionRepresentationTest, ShapeTest)
{
	m_rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");
	m_rigidCollisionRepresentation->setRigidRepresentation(m_rigidRepresentation);
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	m_rigidCollisionRepresentation->initialize(runtime);
	m_rigidCollisionRepresentation->wakeUp();

	EXPECT_EQ(m_sphereShape, m_rigidCollisionRepresentation->getShape());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_SPHERE, m_rigidCollisionRepresentation->getShapeType());

	std::shared_ptr<SurgSim::Math::BoxShape> boxShape = std::make_shared<SurgSim::Math::BoxShape>(1.0, 1.0, 1.0);
	m_rigidCollisionRepresentation->setShape(boxShape);
	EXPECT_EQ(boxShape, m_rigidCollisionRepresentation->getShape());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_BOX, m_rigidCollisionRepresentation->getShapeType());
}

TEST_F(RigidCollisionRepresentationTest, PoseTest)
{
	m_rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");
	m_rigidCollisionRepresentation->setRigidRepresentation(m_rigidRepresentation);

	SurgSim::Math::Quaterniond rotation(1.0, 2.0, 3.0, 4.0);
	SurgSim::Math::Vector3d translation(11.0, 12.0, 13.0);
	SurgSim::Math::RigidTransform3d pose = SurgSim::Math::makeRigidTransform(rotation, translation);
	m_rigidCollisionRepresentation->setLocalPose(pose);
	EXPECT_TRUE(pose.isApprox(m_rigidCollisionRepresentation->getPose()));
}

TEST_F(RigidCollisionRepresentationTest, SerializationTest)
{
	{
		SCOPED_TRACE("RigidCollisionRepresenation must have a shape.");
		auto rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("CollisionRepresentation");

		YAML::Node node;
		// Same as call YAML::convert<SurgSim::Framework::Component>::encode(rigidCollisionRepresentation);
		// Encode RigidCollisionRepresentation as reference has no problem.
		ASSERT_NO_THROW(node = rigidCollisionRepresentation);

		// Encode RigidCollisionRepresentation as concrete object will throw.
		// It's because RigidCollisionRepresentation::getShape() will throw
		// if no shape is assigned and no PhysicsRepresentation is connected.
		ASSERT_ANY_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*rigidCollisionRepresentation));
	}
	{
		SCOPED_TRACE("RigidCollisionRepresenation uses a shape directly.");
		auto rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("CollisionRepresentation");
		std::shared_ptr<SurgSim::Math::Shape> shape = std::make_shared<SurgSim::Math::BoxShape>(0.1, 0.1, 0.1);
		rigidCollisionRepresentation->setValue("Shape", shape);

		YAML::Node node;
		ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*rigidCollisionRepresentation));

		std::shared_ptr<SurgSim::Physics::RigidCollisionRepresentation> newRepresentation;
		ASSERT_NO_THROW(newRepresentation =
							std::dynamic_pointer_cast<SurgSim::Physics::RigidCollisionRepresentation>(
								node.as<std::shared_ptr<SurgSim::Framework::Component>>())
					   );
		EXPECT_EQ("SurgSim::Physics::RigidCollisionRepresentation", newRepresentation->getClassName());
		auto boxShape = std::dynamic_pointer_cast<SurgSim::Math::BoxShape>(shape);
		auto newBoxShape = std::dynamic_pointer_cast<SurgSim::Math::BoxShape>(
							   newRepresentation->getValue<std::shared_ptr<SurgSim::Math::Shape>>("Shape"));
		ASSERT_NE(nullptr, newBoxShape);

		EXPECT_EQ(boxShape->getType(), newRepresentation->getShapeType());
		EXPECT_TRUE(boxShape->getSize().isApprox(newBoxShape->getSize()));
		EXPECT_TRUE(boxShape->getCenter().isApprox(newBoxShape->getCenter()));
		EXPECT_TRUE(boxShape->getSecondMomentOfVolume().isApprox(newBoxShape->getSecondMomentOfVolume()));
		EXPECT_DOUBLE_EQ(boxShape->getVolume(), newBoxShape->getVolume());
	}
	{
		SCOPED_TRACE("RigidCollisionRepresenation uses the shape in PhysicsRepresentation.");
		auto rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("CollisionRepresentation");
		m_rigidRepresentation->setCollisionRepresentation(rigidCollisionRepresentation);
		auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
		rigidCollisionRepresentation->initialize(runtime);
		rigidCollisionRepresentation->wakeUp();

		YAML::Node node;
		ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*rigidCollisionRepresentation));

		std::shared_ptr<SurgSim::Physics::RigidCollisionRepresentation> newRepresentation;
		ASSERT_NO_THROW(newRepresentation =
							std::dynamic_pointer_cast<SurgSim::Physics::RigidCollisionRepresentation>(
								node.as<std::shared_ptr<SurgSim::Framework::Component>>())
					   );
		EXPECT_EQ("SurgSim::Physics::RigidCollisionRepresentation", newRepresentation->getClassName());
		EXPECT_EQ(m_sphereShape->getType(), newRepresentation->getShapeType());

		auto newShpereShape = std::dynamic_pointer_cast<SurgSim::Math::SphereShape>(
								  newRepresentation->getValue<std::shared_ptr<SurgSim::Math::Shape>>("Shape"));
		ASSERT_NE(nullptr, newShpereShape);

		EXPECT_TRUE(m_sphereShape->getCenter().isApprox(newShpereShape->getCenter()));
		EXPECT_TRUE(m_sphereShape->getSecondMomentOfVolume().isApprox(newShpereShape->getSecondMomentOfVolume()));
		EXPECT_DOUBLE_EQ(m_sphereShape->getVolume(), newShpereShape->getVolume());
		EXPECT_DOUBLE_EQ(m_sphereShape->getRadius(), newShpereShape->getRadius());
	}
}

TEST_F(RigidCollisionRepresentationTest, GetPosedShape)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	const std::string fileName = "Geometry/staple_collision.ply";
	auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMeshPlain>();
	EXPECT_NO_THROW(mesh->load(fileName));

	auto originalMesh = std::make_shared<SurgSim::Math::MeshShape>(*mesh);
	auto expectedMesh = std::make_shared<SurgSim::Math::MeshShape>(*mesh);

	auto collisionRepresentation = std::make_shared<RigidCollisionRepresentation>("Collision");
	auto physicsRepresentation = std::make_shared<RigidRepresentation>("Physics");

	physicsRepresentation->setDensity(8050); // Stainless steel (in Kg.m-3)
	physicsRepresentation->setShape(originalMesh);
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);
	ASSERT_TRUE(collisionRepresentation->initialize(runtime));
	ASSERT_TRUE(collisionRepresentation->wakeUp());

	collisionRepresentation->update(dt);

	auto actualMesh =
		std::static_pointer_cast<SurgSim::Math::MeshShape>(collisionRepresentation->getPosedShape().getShape());
	EXPECT_EQ(expectedMesh->getVertices(), actualMesh->getVertices());
	EXPECT_EQ(expectedMesh->getTriangles(), actualMesh->getTriangles());

	RigidTransform3d transform = SurgSim::Math::makeRigidTransform(Vector3d(4.3, 2.1, 6.5),
								 Vector3d(-1.5, 7.5, -2.5),
								 Vector3d(8.7, -4.7, -3.1));
	collisionRepresentation->setLocalPose(transform);
	collisionRepresentation->update(dt);

	actualMesh =
		std::static_pointer_cast<SurgSim::Math::MeshShape>(collisionRepresentation->getPosedShape().getShape());
	expectedMesh->transform(transform);
	expectedMesh->update();
	EXPECT_EQ(expectedMesh->getVertices(), actualMesh->getVertices());
	EXPECT_EQ(expectedMesh->getTriangles(), actualMesh->getTriangles());
}


TEST_F(RigidCollisionRepresentationTest, MeshPoseTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	const std::string fileName = "Geometry/staple_collision.ply";
	auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMeshPlain>();
	EXPECT_NO_THROW(mesh->load(fileName));

	auto originalMesh = std::make_shared<SurgSim::Math::MeshShape>(*mesh);
	auto expectedMesh = std::make_shared<SurgSim::Math::MeshShape>(*mesh);

	auto collisionRepresentation = std::make_shared<RigidCollisionRepresentation>("Collision");
	auto physicsRepresentation = std::make_shared<RigidRepresentation>("Physics");

	physicsRepresentation->setDensity(8050); // Stainless steel (in Kg.m-3)
	physicsRepresentation->setShape(originalMesh);
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);
	collisionRepresentation->setShape(physicsRepresentation->getShape());
	ASSERT_TRUE(collisionRepresentation->initialize(runtime));
	ASSERT_TRUE(collisionRepresentation->wakeUp());

	collisionRepresentation->update(dt);

	auto actualMesh =
		std::static_pointer_cast<SurgSim::Math::MeshShape>(collisionRepresentation->getPosedShape().getShape());
	EXPECT_EQ(expectedMesh->getVertices(), actualMesh->getVertices());
	EXPECT_EQ(expectedMesh->getTriangles(), actualMesh->getTriangles());

	RigidTransform3d transform = SurgSim::Math::makeRigidTransform(Vector3d(4.3, 2.1, 6.5),
		Vector3d(-1.5, 7.5, -2.5),
		Vector3d(8.7, -4.7, -3.1));
	collisionRepresentation->setLocalPose(transform);
	collisionRepresentation->update(dt);

	actualMesh =
		std::static_pointer_cast<SurgSim::Math::MeshShape>(collisionRepresentation->getPosedShape().getShape());
	expectedMesh->transform(transform);
	expectedMesh->update();
	EXPECT_EQ(expectedMesh->getVertices(), actualMesh->getVertices());
	EXPECT_EQ(expectedMesh->getTriangles(), actualMesh->getTriangles());
}

} // namespace Physics
} // namespace SurgSim

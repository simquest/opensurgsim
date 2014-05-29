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

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"

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
		m_rigidRepresentationParameters.setShapeUsedForMassInertia(m_sphereShape);

		m_rigidRepresentation->setInitialParameters(m_rigidRepresentationParameters);
	}

	std::shared_ptr<SurgSim::Math::SphereShape> m_sphereShape;
	std::shared_ptr<SurgSim::Physics::RigidCollisionRepresentation> m_rigidCollisionRepresentation;
	std::shared_ptr<SurgSim::Physics::RigidRepresentation> m_rigidRepresentation;
	SurgSim::Physics::RigidRepresentationParameters m_rigidRepresentationParameters;
};

TEST_F(RigidCollisionRepresentationTest, InitTest)
{
	EXPECT_NO_THROW(RigidCollisionRepresentation("TestRigidCollisionRepresentation"));

	EXPECT_NO_THROW(m_rigidCollisionRepresentation =
					std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation")
	);
}

TEST_F(RigidCollisionRepresentationTest, SetGetRigidRepresentationTest)
{
	m_rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");
	ASSERT_NO_THROW(m_rigidCollisionRepresentation->setRigidRepresentation(m_rigidRepresentation));
	EXPECT_EQ(m_rigidRepresentation, m_rigidCollisionRepresentation->getRigidRepresentation());
}

TEST_F(RigidCollisionRepresentationTest, ShapeTest)
{
	m_rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");
	m_rigidCollisionRepresentation->setRigidRepresentation(m_rigidRepresentation);

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
	m_rigidCollisionRepresentation = std::make_shared<RigidCollisionRepresentation>("RigidCollisionRepresentation");

	YAML::Node node;
	// Same as call YAML::convert<SurgSim::Framework::Component>::encode(m_rigidCollisionRepresentation);
	ASSERT_NO_THROW(node = m_rigidCollisionRepresentation);

	std::shared_ptr<SurgSim::Physics::RigidCollisionRepresentation> newRigidCollisionRepresentation;
	ASSERT_NO_THROW(newRigidCollisionRepresentation =
		std::dynamic_pointer_cast<SurgSim::Physics::RigidCollisionRepresentation>
			(node.as<std::shared_ptr<SurgSim::Framework::Component>>())
		);
}

TEST_F(RigidCollisionRepresentationTest, MeshUpdateTest)
{
	auto applicationData = std::make_shared<SurgSim::Framework::ApplicationData>("config.txt");
	const std::string fileName = "MeshShapeData/staple_collision.ply";
	auto meshShape = std::make_shared<SurgSim::Math::MeshShape>();

	meshShape->setFileName(fileName);
	meshShape->initialize(applicationData);

	RigidRepresentationParameters params;
	params.setDensity(8050); // Stainless steel (in Kg.m-3)
	params.setShapeUsedForMassInertia(meshShape);

	auto collisionRepresentation = std::make_shared<RigidCollisionRepresentation>("Collision");
	auto physicsRepresentation = std::make_shared<RigidRepresentation>("Physics");

	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);
	collisionRepresentation->update(dt);

	auto originalMesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>(*meshShape->getMesh());
	auto expectedMesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>(*meshShape->getMesh());
	auto actualMesh
		= std::static_pointer_cast<SurgSim::Math::MeshShape>(collisionRepresentation->getShape())->getMesh();

	EXPECT_EQ(expectedMesh->getVertices(), actualMesh->getVertices());
	EXPECT_EQ(expectedMesh->getTriangles(), actualMesh->getTriangles());

	RigidTransform3d transform = SurgSim::Math::makeRigidTransform(Vector3d(4.3, 2.1, 6.5),
																   Vector3d(-1.5, 7.5, -2.5),
																   Vector3d(8.7, -4.7, -3.1));

	//physicsRepresentation->setLocalPose(transform);
	//collisionRepresentation->update(dt);

	std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(collisionRepresentation->getShape())->setPose(transform);
	expectedMesh->copyWithTransform(transform, *originalMesh);

	EXPECT_EQ(expectedMesh->getVertices(), actualMesh->getVertices());
	EXPECT_EQ(expectedMesh->getTriangles(), actualMesh->getTriangles());
}

} // namespace Physics
} // namespace SurgSim

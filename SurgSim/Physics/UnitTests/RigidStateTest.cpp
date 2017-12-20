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

#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/PhysicsConvert.h"
#include "SurgSim/Physics/RigidState.h"

struct RigidStateTest : public ::testing::Test
{
	void SetUp()
	{
		quaterniond = SurgSim::Math::Quaterniond(0.5, 0.4, 0.3, 0.2);
		quaterniond.normalize();
		translation = SurgSim::Math::Vector3d(5.2, -6.13, 4.12356);
		pose = SurgSim::Math::makeRigidTransform(quaterniond, translation);
		linearVelocity = SurgSim::Math::Vector3d(2, -3.1, -2.75);
		angularVelocity = SurgSim::Math::Vector3d(5, -10, 21.5);
		id4x4 = SurgSim::Math::RigidTransform3d::Identity();
	}

	SurgSim::Math::UnalignedQuaterniond quaterniond;
	SurgSim::Math::Vector3d translation;
	SurgSim::Math::UnalignedRigidTransform3d pose;

	SurgSim::Math::Vector3d linearVelocity;
	SurgSim::Math::Vector3d angularVelocity;

	SurgSim::Math::UnalignedRigidTransform3d id4x4;
};

TEST_F(RigidStateTest, ConstructorTest)
{
	ASSERT_NO_THROW(SurgSim::Physics::RigidState rigidRepresentationState);
}

TEST_F(RigidStateTest, DefaultValueTest)
{
	// Create the base rigid representation state
	std::shared_ptr<SurgSim::Physics::RigidState> rigidRepresentationState;
	rigidRepresentationState = std::make_shared<SurgSim::Physics::RigidState>();

	// Linear velocity [default = (0 0 0)]
	EXPECT_TRUE(rigidRepresentationState->getLinearVelocity().isZero());
	// Angular velocity [default = (0 0 0)]
	EXPECT_TRUE(rigidRepresentationState->getAngularVelocity().isZero());
	// Pose [default = Identity]
	EXPECT_TRUE(rigidRepresentationState->getPose().isApprox(id4x4));
}

TEST_F(RigidStateTest, ResetTest)
{
	// Create the base rigid representation state
	std::shared_ptr<SurgSim::Physics::RigidState> rigidRepresentationState;
	rigidRepresentationState = std::make_shared<SurgSim::Physics::RigidState>();

	rigidRepresentationState->setLinearVelocity(linearVelocity);
	rigidRepresentationState->setAngularVelocity(angularVelocity);
	rigidRepresentationState->setPose(pose);

	// Reset the rigid representation state to default values
	rigidRepresentationState->reset();

	// Test Linear velocity has been reset to (0 0 0)
	EXPECT_TRUE(rigidRepresentationState->getLinearVelocity().isZero());
	// Test Angular velocity has been reset to (0 0 0)
	EXPECT_TRUE(rigidRepresentationState->getAngularVelocity().isZero());
	// Test pose has been reset to Identity
	EXPECT_TRUE(rigidRepresentationState->getPose().isApprox(id4x4));
}

TEST_F(RigidStateTest, SetGetTest)
{
	// Create the base rigid representation state
	std::shared_ptr<SurgSim::Physics::RigidState> rigidRepresentationState;
	rigidRepresentationState = std::make_shared<SurgSim::Physics::RigidState>();

	// Get/Set linear velocity
	rigidRepresentationState->setLinearVelocity(linearVelocity);
	EXPECT_TRUE(linearVelocity.isApprox(rigidRepresentationState->getLinearVelocity()));
	rigidRepresentationState->setLinearVelocity(SurgSim::Math::Vector3d::Zero());
	EXPECT_TRUE(rigidRepresentationState->getLinearVelocity().isZero());

	// Get/Set angular velocity
	rigidRepresentationState->setAngularVelocity(angularVelocity);
	EXPECT_TRUE(angularVelocity.isApprox(rigidRepresentationState->getAngularVelocity()));
	rigidRepresentationState->setAngularVelocity(SurgSim::Math::Vector3d::Zero());
	EXPECT_TRUE(rigidRepresentationState->getAngularVelocity().isZero());

	// Get/Set pose
	rigidRepresentationState->setPose(pose);
	EXPECT_TRUE(rigidRepresentationState->getPose().isApprox(pose));
	rigidRepresentationState->setPose(id4x4);
	EXPECT_TRUE(rigidRepresentationState->getPose().isApprox(id4x4));
}

TEST_F(RigidStateTest, SerializationTest)
{
	SurgSim::Physics::RigidState rigidRepresentationState;

	rigidRepresentationState.setValue("Pose", pose);
	rigidRepresentationState.setValue("LinearVelocity", linearVelocity);
	rigidRepresentationState.setValue("AngularVelocity", angularVelocity);

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Physics::RigidState>::encode(rigidRepresentationState));
	EXPECT_EQ(1u, node.size());

	SurgSim::Physics::RigidState newRigidRepresentationState;
	ASSERT_NO_THROW(newRigidRepresentationState = node.as<SurgSim::Physics::RigidState>());

	EXPECT_TRUE(pose.isApprox(newRigidRepresentationState.getValue<SurgSim::Math::UnalignedRigidTransform3d>("Pose")));
	EXPECT_TRUE(linearVelocity.isApprox(
					newRigidRepresentationState.getValue<SurgSim::Math::Vector3d>("LinearVelocity")));
	EXPECT_TRUE(angularVelocity.isApprox(
					newRigidRepresentationState.getValue<SurgSim::Math::Vector3d>("AngularVelocity")));
}
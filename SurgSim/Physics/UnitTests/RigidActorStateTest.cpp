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

#include <SurgSim/Physics/Actors/RigidActorState.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

TEST(RigidActorStateTest, ConstructorTest)
{
	ASSERT_NO_THROW( {SurgSim::Physics::RigidActorState rigidActorState;});
}

TEST(RigidActorStateTest, DefaultValueTest)
{
	const SurgSim::Math::Vector3d nullVector = SurgSim::Math::Vector3d::Zero();
	const SurgSim::Math::RigidTransform3d id4x4 = SurgSim::Math::RigidTransform3d::Identity();

	// Create the base rigid actor state
	std::shared_ptr<SurgSim::Physics::RigidActorState> rigidActorState;
	rigidActorState = std::make_shared<SurgSim::Physics::RigidActorState>();

	// Linear velocity [default = (0 0 0)]
	EXPECT_EQ(nullVector, rigidActorState->getLinearVelocity());
	// Angular velocity [default = (0 0 0)]
	EXPECT_EQ(nullVector, rigidActorState->getAngularVelocity());
	// Pose [default = Identity]
	EXPECT_TRUE(rigidActorState->getPose().isApprox(id4x4));
}

TEST(RigidActorStateTest, ResetTest)
{
	// Local useful variables
	SurgSim::Math::Quaterniond q(0.5, 0.4, 0.3, 0.2);
	q.normalize();
	SurgSim::Math::Vector3d t(5.2, -6.13, 4.12356);
	const SurgSim::Math::RigidTransform3d pose = SurgSim::Math::makeRigidTransform(q, t);
	const SurgSim::Math::Vector3d linearVelocity(2, -3.1, -2.75);
	const SurgSim::Math::Vector3d angularVelocity(5, -10, 21.5);
	const SurgSim::Math::RigidTransform3d id4x4 = SurgSim::Math::RigidTransform3d::Identity();
	const SurgSim::Math::Vector3d nullVector = SurgSim::Math::Vector3d::Zero();

	// Create the base rigid actor state
	std::shared_ptr<SurgSim::Physics::RigidActorState> rigidActorState;
	rigidActorState = std::make_shared<SurgSim::Physics::RigidActorState>();

	// Set linear velocity
	rigidActorState->setLinearVelocity(linearVelocity);
	// Set angular velocity
	rigidActorState->setAngularVelocity(angularVelocity);
	// Set pose
	rigidActorState->setPose(pose);

	// Reset the rigid actor state to default values
	rigidActorState->reset();

	// Test Linear velocity has been reset to (0 0 0)
	EXPECT_EQ(nullVector, rigidActorState->getLinearVelocity());
	// Test Angular velocity has been reset to (0 0 0)
	EXPECT_EQ(nullVector, rigidActorState->getAngularVelocity());
	// Test pose has been reset to Identity
	EXPECT_TRUE(rigidActorState->getPose().isApprox(id4x4));
}

TEST(RigidActorStateTest, SetGetTest)
{
	// Local useful variables
	SurgSim::Math::Quaterniond q(0.5, 0.4, 0.3, 0.2);
	q.normalize();
	SurgSim::Math::Vector3d t(5.2, -6.13, 4.12356);
	const SurgSim::Math::RigidTransform3d pose = SurgSim::Math::makeRigidTransform(q, t);
	const SurgSim::Math::RigidTransform3d id4x4 = SurgSim::Math::RigidTransform3d::Identity();
	const SurgSim::Math::Vector3d linearVelocity(2, -3.1, -2.75);
	const SurgSim::Math::Vector3d angularVelocity(5, -10, 21.5);
	const SurgSim::Math::Vector3d nullVector = SurgSim::Math::Vector3d::Zero();

	// Create the base rigid actor state
	std::shared_ptr<SurgSim::Physics::RigidActorState> rigidActorState;
	rigidActorState = std::make_shared<SurgSim::Physics::RigidActorState>();

	// Get/Set linear velocity
	rigidActorState->setLinearVelocity(linearVelocity);
	EXPECT_EQ(linearVelocity, rigidActorState->getLinearVelocity());
	rigidActorState->setLinearVelocity(nullVector);
	EXPECT_EQ(nullVector, rigidActorState->getLinearVelocity());

	// Get/Set angular velocity
	rigidActorState->setAngularVelocity(angularVelocity);
	EXPECT_EQ(angularVelocity, rigidActorState->getAngularVelocity());
	rigidActorState->setAngularVelocity(nullVector);
	EXPECT_EQ(nullVector, rigidActorState->getAngularVelocity());

	// Get/Set pose
	rigidActorState->setPose(pose);
	EXPECT_TRUE(rigidActorState->getPose().isApprox(pose));
	rigidActorState->setPose(id4x4);
	EXPECT_TRUE(rigidActorState->getPose().isApprox(id4x4));
}

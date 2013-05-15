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

#include <string>

#include <SurgSim/Physics/Actors/FixedActor.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>

using namespace SurgSim::Physics;

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;

class FixedActorTest : public ::testing::Test
{
public:
	void SetUp()
	{
		gGravity << 0.0, -9.81, 0.0;
		gNoGravity.setZero();

		Quaterniond q;
		Vector3d t;

		q.coeffs().setRandom();
		q.normalize();
		t.setRandom();
		initialTransformation = SurgSim::Math::makeRigidTransform(q, t);

		do
		{
			q.coeffs().setRandom();
			q.normalize();
			t.setRandom();
			currentTransformation = SurgSim::Math::makeRigidTransform(q, t);
		} while (initialTransformation.isApprox(currentTransformation));

		identityTransformation.setIdentity();
	}

	void TearDown()
	{
	}

	// Gravity vector (normal version)
	Vector3d gGravity;
	// Gravity vector (version without gravity)
	Vector3d gNoGravity;

	// Fixed actor initialization pose
	RigidTransform3d initialTransformation;

	// Fixed actor current pose
	RigidTransform3d currentTransformation;

	// Identity pose (no translation/rotation)
	RigidTransform3d identityTransformation;
};

TEST_F(FixedActorTest, ConstructorTest)
{
	ASSERT_NO_THROW( {FixedActor fixedActor(std::string("FixedActor"));});
}

TEST_F(FixedActorTest, ResetStateTest)
{
	// Create the rigid body
	std::shared_ptr<FixedActor> fixedActor = std::make_shared<FixedActor>(std::string("FixedActor"));

	fixedActor->setIsActive(false);
	fixedActor->setIsGravityEnabled(false);
	fixedActor->setGravity(gNoGravity);
	fixedActor->setInitialPose(initialTransformation);
	EXPECT_TRUE(fixedActor->getPose().isApprox(fixedActor->getInitialPose()));
	EXPECT_TRUE(fixedActor->getPreviousPose().isApprox(fixedActor->getInitialPose()));
	fixedActor->setPose(currentTransformation);
	EXPECT_FALSE(fixedActor->getPose().isApprox(fixedActor->getInitialPose()));
	EXPECT_TRUE(fixedActor->getPreviousPose().isApprox(fixedActor->getInitialPose()));

	// reset the actor (NOT THE FIXED ACTOR, test polymorphism)
	std::static_pointer_cast<Actor>(fixedActor)->resetState();

	// isActive flag [default = true]
	EXPECT_TRUE(fixedActor->isActive());
	// isGravityEnable flag [default = true]
	EXPECT_TRUE(fixedActor->isGravityEnabled());
	// gravity vector [default = (0 -9.81 0)]
	EXPECT_EQ(gGravity, fixedActor->getGravity());
	// The current rigid state should be exactly the initial rigid state
	EXPECT_TRUE(fixedActor->getPose().isApprox(fixedActor->getInitialPose()));
	EXPECT_TRUE(fixedActor->getPose().isApprox(initialTransformation));
	EXPECT_TRUE(fixedActor->getInitialPose().isApprox(initialTransformation));
	// The previous rigid state should be exactly the initial rigid state
	EXPECT_TRUE(fixedActor->getPreviousPose().isApprox(fixedActor->getInitialPose()));
	EXPECT_TRUE(fixedActor->getPreviousPose().isApprox(initialTransformation));
}

TEST_F(FixedActorTest, SetGetAndDefaultValueTest)
{
	// Create the fixed actor
	std::shared_ptr<FixedActor> fixedActor = std::make_shared<FixedActor>(std::string("FixedActor"));

	// Get/Set active flag [default = true]
	EXPECT_TRUE(fixedActor->isActive());
	fixedActor->setIsActive(false);
	ASSERT_FALSE(fixedActor->isActive());
	fixedActor->setIsActive(true);
	ASSERT_TRUE(fixedActor->isActive());

	// Get numDof = 0
	ASSERT_EQ(0, fixedActor->getNumDof());

	// Set/Get isGravityEnabled [default = true]
	EXPECT_TRUE(fixedActor->isGravityEnabled());
	fixedActor->setIsGravityEnabled(false);
	ASSERT_FALSE(fixedActor->isGravityEnabled());
	fixedActor->setIsGravityEnabled(true);
	ASSERT_TRUE(fixedActor->isGravityEnabled());

	// Set/Get gravity [default = (0 -9.81 0)]
	EXPECT_EQ(gGravity, fixedActor->getGravity());
	fixedActor->setGravity(gNoGravity);
	ASSERT_EQ(gNoGravity, fixedActor->getGravity());
	fixedActor->setGravity(gGravity);
	ASSERT_EQ(gGravity, fixedActor->getGravity());

	// Set/Get current pose [default = no translation, no rotation]
	EXPECT_TRUE(identityTransformation.isApprox(fixedActor->getPose()));
	fixedActor->setPose(currentTransformation);
	ASSERT_TRUE(currentTransformation.isApprox(fixedActor->getPose()));
	fixedActor->setPose(identityTransformation);
	ASSERT_TRUE(identityTransformation.isApprox(fixedActor->getPose()));

	// Set/Get initial pose [default = no translation, no rotation]
	EXPECT_TRUE(identityTransformation.isApprox(fixedActor->getInitialPose()));
	fixedActor->setInitialPose(initialTransformation);
	ASSERT_TRUE(initialTransformation.isApprox(fixedActor->getInitialPose()));
	fixedActor->setInitialPose(identityTransformation);
	ASSERT_TRUE(identityTransformation.isApprox(fixedActor->getInitialPose()));
}

TEST_F(FixedActorTest, UpdateTest)
{
	// Create the fixed actor
	std::shared_ptr<FixedActor> fixedActor = std::make_shared<FixedActor>(std::string("FixedActor"));

	double dt = 1.0;

	fixedActor->setInitialPose(initialTransformation);
	fixedActor->setPose(currentTransformation);
	// This should simply backup the current transformation into the previous
	fixedActor->update(dt);
	EXPECT_TRUE(fixedActor->getPose().isApprox(fixedActor->getPreviousPose()));
	EXPECT_FALSE(fixedActor->getPose().isApprox(fixedActor->getInitialPose()));
	EXPECT_FALSE(fixedActor->getPreviousPose().isApprox(fixedActor->getInitialPose()));
	fixedActor->setPose(identityTransformation);
	EXPECT_FALSE(fixedActor->getPose().isApprox(fixedActor->getPreviousPose()));
}

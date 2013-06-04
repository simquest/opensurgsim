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

#include <SurgSim/Physics/FixedActor.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using namespace SurgSim::Physics;

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

class FixedActorTest : public ::testing::Test
{
public:
	void SetUp()
	{
		Quaterniond q;
		Vector3d t;

		q.coeffs().setRandom();
		q.normalize();
		t.setRandom();
		m_initialTransformation = SurgSim::Math::makeRigidTransform(q, t);

		do
		{
			q.coeffs().setRandom();
			q.normalize();
			t.setRandom();
			m_currentTransformation = SurgSim::Math::makeRigidTransform(q, t);
		} while (m_initialTransformation.isApprox(m_currentTransformation));

		m_identityTransformation.setIdentity();
	}

	void TearDown()
	{
	}

	// Fixed actor initialization pose
	RigidTransform3d m_initialTransformation;

	// Fixed actor current pose
	RigidTransform3d m_currentTransformation;

	// Identity pose (no translation/rotation)
	RigidTransform3d m_identityTransformation;
};

TEST_F(FixedActorTest, ConstructorTest)
{
	ASSERT_NO_THROW( {FixedActor fixedActor("FixedActor");});
}

TEST_F(FixedActorTest, ResetStateTest)
{
	// Create the rigid body
	std::shared_ptr<FixedActor> fixedActor = std::make_shared<FixedActor>("FixedActor");

	fixedActor->setIsActive(false);
	fixedActor->setIsGravityEnabled(false);
	fixedActor->setInitialPose(m_initialTransformation);
	EXPECT_TRUE(fixedActor->getPose().isApprox(fixedActor->getInitialPose()));
	EXPECT_TRUE(fixedActor->getPreviousPose().isApprox(fixedActor->getInitialPose()));
	fixedActor->setPose(m_currentTransformation);
	EXPECT_FALSE(fixedActor->getPose().isApprox(fixedActor->getInitialPose()));
	EXPECT_TRUE(fixedActor->getPreviousPose().isApprox(fixedActor->getInitialPose()));

	std::shared_ptr<Actor> actor = fixedActor;
	// reset the actor (NOT THE FIXED ACTOR, test polymorphism)
	actor->resetState();

	// isActive flag unchanged
	EXPECT_FALSE(fixedActor->isActive());
	// isGravityEnable flag unchanged
	EXPECT_FALSE(fixedActor->isGravityEnabled());
	// The current rigid state should be exactly the initial rigid state
	EXPECT_TRUE(fixedActor->getPose().isApprox(fixedActor->getInitialPose()));
	EXPECT_TRUE(fixedActor->getPose().isApprox(m_initialTransformation));
	EXPECT_TRUE(fixedActor->getInitialPose().isApprox(m_initialTransformation));
	// The previous rigid state should be exactly the initial rigid state
	EXPECT_TRUE(fixedActor->getPreviousPose().isApprox(fixedActor->getInitialPose()));
	EXPECT_TRUE(fixedActor->getPreviousPose().isApprox(m_initialTransformation));
}

TEST_F(FixedActorTest, SetGetAndDefaultValueTest)
{
	// Create the fixed actor
	std::shared_ptr<FixedActor> fixedActor = std::make_shared<FixedActor>("FixedActor");

	// Get/Set active flag [default = true]
	EXPECT_TRUE(fixedActor->isActive());
	fixedActor->setIsActive(false);
	ASSERT_FALSE(fixedActor->isActive());
	fixedActor->setIsActive(true);
	ASSERT_TRUE(fixedActor->isActive());

	// Get numDof = 0
	ASSERT_EQ(0u, fixedActor->getNumDof());

	// Set/Get isGravityEnabled [default = true]
	EXPECT_TRUE(fixedActor->isGravityEnabled());
	fixedActor->setIsGravityEnabled(false);
	ASSERT_FALSE(fixedActor->isGravityEnabled());
	fixedActor->setIsGravityEnabled(true);
	ASSERT_TRUE(fixedActor->isGravityEnabled());

	// Set/Get current pose [default = no translation, no rotation]
	EXPECT_TRUE(m_identityTransformation.isApprox(fixedActor->getPose()));
	fixedActor->setPose(m_currentTransformation);
	ASSERT_TRUE(m_currentTransformation.isApprox(fixedActor->getPose()));
	fixedActor->setPose(m_identityTransformation);
	ASSERT_TRUE(m_identityTransformation.isApprox(fixedActor->getPose()));

	// Set/Get initial pose [default = no translation, no rotation]
	EXPECT_TRUE(m_identityTransformation.isApprox(fixedActor->getInitialPose()));
	fixedActor->setInitialPose(m_initialTransformation);
	ASSERT_TRUE(m_initialTransformation.isApprox(fixedActor->getInitialPose()));
	fixedActor->setInitialPose(m_identityTransformation);
	ASSERT_TRUE(m_identityTransformation.isApprox(fixedActor->getInitialPose()));
}

TEST_F(FixedActorTest, UpdateTest)
{
	// Create the fixed actor
	std::shared_ptr<FixedActor> fixedActor = std::make_shared<FixedActor>("FixedActor");

	double dt = 1.0;

	fixedActor->setInitialPose(m_initialTransformation);
	fixedActor->setPose(m_currentTransformation);

	// This should simply backup the current transformation into the previous
	fixedActor->update(dt);

	EXPECT_TRUE(fixedActor->getPose().isApprox(fixedActor->getPreviousPose()));
	EXPECT_FALSE(fixedActor->getPose().isApprox(fixedActor->getInitialPose()));
	EXPECT_FALSE(fixedActor->getPreviousPose().isApprox(fixedActor->getInitialPose()));
	fixedActor->setPose(m_identityTransformation);
	EXPECT_FALSE(fixedActor->getPose().isApprox(fixedActor->getPreviousPose()));
}

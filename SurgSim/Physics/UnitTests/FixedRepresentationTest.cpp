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

#include <SurgSim/Physics/FixedRepresentation.h>
using SurgSim::Physics::Representation;
using SurgSim::Physics::FixedRepresentation;

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

class FixedRepresentationTest : public ::testing::Test
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

	// Fixed representation initialization pose
	RigidTransform3d m_initialTransformation;

	// Fixed representation current pose
	RigidTransform3d m_currentTransformation;

	// Identity pose (no translation/rotation)
	RigidTransform3d m_identityTransformation;
};

TEST_F(FixedRepresentationTest, ConstructorTest)
{
	ASSERT_NO_THROW( {FixedRepresentation fixedRepresentation("FixedRepresentation");});
}

TEST_F(FixedRepresentationTest, ResetStateTest)
{
	// Create the rigid body
	std::shared_ptr<FixedRepresentation> fixedRepresentation =
		std::make_shared<FixedRepresentation>("FixedRepresentation");

	fixedRepresentation->setIsActive(false);
	fixedRepresentation->setIsGravityEnabled(false);
	fixedRepresentation->setInitialPose(m_initialTransformation);
	// Initial = Current = Previous = m_initialTransformation
	EXPECT_TRUE(fixedRepresentation->getPose().isApprox(fixedRepresentation->getInitialPose()));
	EXPECT_TRUE(fixedRepresentation->getPreviousPose().isApprox(fixedRepresentation->getInitialPose()));
	EXPECT_TRUE(fixedRepresentation->getPose().isApprox(m_initialTransformation));
	EXPECT_TRUE(fixedRepresentation->getPreviousPose().isApprox(m_initialTransformation));
	EXPECT_TRUE(fixedRepresentation->getInitialPose().isApprox(m_initialTransformation));

	fixedRepresentation->setPose(m_currentTransformation);
	// setCurrent is supposed to backup current in previous and set current
	// Therefore it should not affect initial
	// Initial = Previous = m_initialTransformation
	// Current = m_currentTransformation
	EXPECT_FALSE(fixedRepresentation->getPose().isApprox(fixedRepresentation->getInitialPose()));
	EXPECT_TRUE(fixedRepresentation->getPreviousPose().isApprox(fixedRepresentation->getInitialPose()));
	EXPECT_FALSE(fixedRepresentation->getPreviousPose().isApprox(fixedRepresentation->getPose()));
	EXPECT_TRUE(fixedRepresentation->getPose().isApprox(m_currentTransformation));

	fixedRepresentation->setPose(m_currentTransformation);
	// setCurrent is supposed to backup current in previous and set current
	// Therefore it should not affect initial
	// Initial = m_initialTransformation
	// Previous = Current = m_currentTransformation
	EXPECT_FALSE(fixedRepresentation->getPose().isApprox(fixedRepresentation->getInitialPose()));
	EXPECT_FALSE(fixedRepresentation->getPreviousPose().isApprox(fixedRepresentation->getInitialPose()));
	EXPECT_TRUE(fixedRepresentation->getPreviousPose().isApprox(fixedRepresentation->getPose()));
	EXPECT_TRUE(fixedRepresentation->getPose().isApprox(m_currentTransformation));

	std::shared_ptr<Representation> representation = fixedRepresentation;
	// reset the representation (NOT THE FIXED REPRESENTATION, test polymorphism)
	representation->resetState();

	// isActive flag unchanged
	EXPECT_FALSE(fixedRepresentation->isActive());
	// isGravityEnable flag unchanged
	EXPECT_FALSE(fixedRepresentation->isGravityEnabled());
	// The current rigid state should be exactly the initial rigid state
	EXPECT_TRUE(fixedRepresentation->getPose().isApprox(fixedRepresentation->getInitialPose()));
	EXPECT_TRUE(fixedRepresentation->getPose().isApprox(m_initialTransformation));
	EXPECT_TRUE(fixedRepresentation->getInitialPose().isApprox(m_initialTransformation));
	// The previous rigid state should be exactly the initial rigid state
	EXPECT_TRUE(fixedRepresentation->getPreviousPose().isApprox(fixedRepresentation->getInitialPose()));
	EXPECT_TRUE(fixedRepresentation->getPreviousPose().isApprox(m_initialTransformation));
}

TEST_F(FixedRepresentationTest, SetGetAndDefaultValueTest)
{
	// Create the fixed representation
	std::shared_ptr<FixedRepresentation> fixedRepresentation =
		std::make_shared<FixedRepresentation>("FixedRepresentation");

	// Get/Set active flag [default = true]
	EXPECT_TRUE(fixedRepresentation->isActive());
	fixedRepresentation->setIsActive(false);
	ASSERT_FALSE(fixedRepresentation->isActive());
	fixedRepresentation->setIsActive(true);
	ASSERT_TRUE(fixedRepresentation->isActive());

	// Get numDof = 0
	ASSERT_EQ(0u, fixedRepresentation->getNumDof());

	// Set/Get isGravityEnabled [default = true]
	EXPECT_TRUE(fixedRepresentation->isGravityEnabled());
	fixedRepresentation->setIsGravityEnabled(false);
	ASSERT_FALSE(fixedRepresentation->isGravityEnabled());
	fixedRepresentation->setIsGravityEnabled(true);
	ASSERT_TRUE(fixedRepresentation->isGravityEnabled());

	// Set/Get current pose [default = no translation, no rotation]
	EXPECT_TRUE(m_identityTransformation.isApprox(fixedRepresentation->getPose()));
	fixedRepresentation->setPose(m_currentTransformation);
	fixedRepresentation->afterUpdate(1.0); // afterUpdate backs up current into final
	ASSERT_TRUE(m_currentTransformation.isApprox(fixedRepresentation->getPose()));
	fixedRepresentation->setPose(m_identityTransformation);
	fixedRepresentation->afterUpdate(1.0); // afterUpdate backs up current into final
	ASSERT_TRUE(m_identityTransformation.isApprox(fixedRepresentation->getPose()));

	// Set/Get initial pose [default = no translation, no rotation]
	EXPECT_TRUE(m_identityTransformation.isApprox(fixedRepresentation->getInitialPose()));
	fixedRepresentation->setInitialPose(m_initialTransformation);
	ASSERT_TRUE(m_initialTransformation.isApprox(fixedRepresentation->getInitialPose()));
	fixedRepresentation->setInitialPose(m_identityTransformation);
	ASSERT_TRUE(m_identityTransformation.isApprox(fixedRepresentation->getInitialPose()));
}

TEST_F(FixedRepresentationTest, UpdateTest)
{
	// Create the fixed representation
	std::shared_ptr<FixedRepresentation> fixedRepresentation =
		std::make_shared<FixedRepresentation>("FixedRepresentation");

	double dt = 1.0;

	// Sets Initial state and reset all other state with initial state
	fixedRepresentation->setInitialPose(m_initialTransformation);
	// Backs up current state into previous state and set the current
	fixedRepresentation->setPose(m_currentTransformation);

	// Does not do anything
	fixedRepresentation->beforeUpdate(dt);
	// Does not do anything
	fixedRepresentation->update(dt);
	// Backs up current state into final state
	fixedRepresentation->afterUpdate(dt);

	EXPECT_FALSE(fixedRepresentation->getPose().isApprox(fixedRepresentation->getPreviousPose()));
	EXPECT_FALSE(fixedRepresentation->getPose().isApprox(fixedRepresentation->getInitialPose()));
	EXPECT_TRUE(fixedRepresentation->getPreviousPose().isApprox(fixedRepresentation->getInitialPose()));
}


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
#include <memory>

#include <SurgSim/Physics/RigidActorBaseState.h>
using SurgSim::Physics::RigidActorBaseState;

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

class RigidActorBaseStateTest : public ::testing::Test
{
public:
	void SetUp()
	{
		Quaterniond q;
		Vector3d t;

		q.coeffs().setRandom();
		q.normalize();
		t.setRandom();
		m_currentTransformation = SurgSim::Math::makeRigidTransform(q, t);

		m_identityTransformation.setIdentity();
	}

	void TearDown()
	{
	}

	// Rigid actor current pose
	RigidTransform3d m_currentTransformation;

	// Identity pose (no translation/rotation)
	RigidTransform3d m_identityTransformation;
};

TEST_F(RigidActorBaseStateTest, ConstructorTest)
{
	ASSERT_NO_THROW( {RigidActorBaseState rigidActorBaseState;});
}

TEST_F(RigidActorBaseStateTest, DefaultValueTest)
{
	/// Create the base rigid actor state
	std::shared_ptr<RigidActorBaseState> rigidActorBaseState = std::make_shared<RigidActorBaseState>();

	/// Pose [default = identity]
	EXPECT_TRUE(rigidActorBaseState->getPose().isApprox(m_identityTransformation));
}

TEST_F(RigidActorBaseStateTest, ResetTest)
{
	/// Create the base rigid actor state
	std::shared_ptr<RigidActorBaseState> rigidActorBaseState = std::make_shared<RigidActorBaseState>();

	rigidActorBaseState->setPose(m_currentTransformation);
	EXPECT_FALSE(rigidActorBaseState->getPose().isApprox(m_identityTransformation));
	// Reset should reset the pose to identity
	rigidActorBaseState->reset();
	EXPECT_TRUE(rigidActorBaseState->getPose().isApprox(m_identityTransformation));
}

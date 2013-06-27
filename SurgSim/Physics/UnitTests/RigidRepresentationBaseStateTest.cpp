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

#include <SurgSim/Physics/RigidRepresentationBaseState.h>
using SurgSim::Physics::RigidRepresentationBaseState;

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

class RigidRepresentationBaseStateTest : public ::testing::Test
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

	// Rigid representation current pose
	RigidTransform3d m_currentTransformation;

	// Identity pose (no translation/rotation)
	RigidTransform3d m_identityTransformation;
};

TEST_F(RigidRepresentationBaseStateTest, ConstructorTest)
{
	ASSERT_NO_THROW( {RigidRepresentationBaseState rigidRepresentationBaseState;});
}

TEST_F(RigidRepresentationBaseStateTest, DefaultValueTest)
{
	/// Create the base rigid representation state
	std::shared_ptr<RigidRepresentationBaseState> rigidRepresentationBaseState =
		std::make_shared<RigidRepresentationBaseState>();

	/// Pose [default = identity]
	EXPECT_TRUE(rigidRepresentationBaseState->getPose().isApprox(m_identityTransformation));
}

TEST_F(RigidRepresentationBaseStateTest, ResetTest)
{
	/// Create the base rigid representation state
	std::shared_ptr<RigidRepresentationBaseState> rigidRepresentationBaseState = std::make_shared<RigidRepresentationBaseState>();

	rigidRepresentationBaseState->setPose(m_currentTransformation);
	EXPECT_FALSE(rigidRepresentationBaseState->getPose().isApprox(m_identityTransformation));
	// Reset should reset the pose to identity
	rigidRepresentationBaseState->reset();
	EXPECT_TRUE(rigidRepresentationBaseState->getPose().isApprox(m_identityTransformation));
}

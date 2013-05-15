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

#include <SurgSim/Physics/Actors/Actor.h>
#include <SurgSim/Math/Vector.h>

using SurgSim::Physics::Actor;
using SurgSim::Math::Vector3d;

class ActorTest : public ::testing::Test
{
public:
	void SetUp()
	{
		m_Gravity << 0.0, -9.81, 0.0;
		m_zeroGravity.setZero();
	}

	void TearDown()
	{
	}

	// Gravity vector (normal version)
	Vector3d m_Gravity;

	// Gravity vector (version without gravity)
	Vector3d m_zeroGravity;
};


TEST_F(ActorTest, ConstructorTest)
{
	ASSERT_NO_THROW({Actor actor("Actor");});
}

TEST_F(ActorTest, ResetStateTest)
{
	// Create the rigid body
	std::shared_ptr<Actor> actor = std::make_shared<Actor>("Actor");

	// Set the actor state to non default values
	actor->setIsActive(false);
	actor->setIsGravityEnabled(false);
	actor->setGravity(m_zeroGravity);

	// Reset the actor state to the default values
	actor->resetState();

	// isActive flag [default = true]
	EXPECT_TRUE(actor->isActive());
	// isGravityEnable flag [default = true]
	EXPECT_TRUE(actor->isGravityEnabled());
	// gravity vector [default = (0 -9.81 0)]
	EXPECT_EQ(m_Gravity, actor->getGravity());
}

TEST_F(ActorTest, SetGetAndDefaultValueTest)
{
	/// Create the actor
	std::shared_ptr<Actor> actor = std::make_shared<Actor>("Actor");

	/// Get/Set active flag [default = true]
	EXPECT_TRUE(actor->isActive());
	actor->setIsActive(false);
	ASSERT_FALSE(actor->isActive());
	actor->setIsActive(true);
	ASSERT_TRUE(actor->isActive());

	/// Get numDof = 0
	ASSERT_EQ(0, actor->getNumDof());

	/// Set/Get isGravityEnabled [default = true]
	EXPECT_TRUE(actor->isGravityEnabled());
	actor->setIsGravityEnabled(false);
	ASSERT_FALSE(actor->isGravityEnabled());
	actor->setIsGravityEnabled(true);
	ASSERT_TRUE(actor->isGravityEnabled());

	/// Set/Get gravity [default = (0 -9.81 0)]
	EXPECT_EQ(m_Gravity, actor->getGravity());
	actor->setGravity(m_zeroGravity);
	ASSERT_EQ(m_zeroGravity, actor->getGravity());
	actor->setGravity(m_Gravity);
	ASSERT_EQ(m_Gravity, actor->getGravity());
}

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

#include <SurgSim/Physics/Actor.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

using SurgSim::Physics::Actor;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

/// Concrete actor class for testing
class MockActor : public Actor
{
public:
	/// Constructor
	/// \name	Name of the actor
	MockActor(const std::string& name) : Actor(name)
	{
	}

	/// Sets the initial pose
	/// \param pose The initial pose to set the MockActor to
	void setInitialPose(const RigidTransform3d& pose)
	{
		m_initialPose = pose;
	}

	/// Gets the initial pose of the actor
	/// \return The initial pose of this MockActor
	const RigidTransform3d& getInitialPose() const
	{
		return m_initialPose;
	}

	/// Sets the current pose of the actor
	/// \param pose The current pose of this MockActor
	void setCurrentPose(const RigidTransform3d& pose)
	{
		m_currentPose = pose;
	}

	/// Returns the previous pose of the actor
	/// \return The previous pose of this MockActor
	const RigidTransform3d& getPreviousPose() const
	{
		return m_previousPose;
	}

	/// Returns the final pose of the actor
	/// \return The final pose of this MockActor
	const RigidTransform3d& getFinalPose() const
	{
		return m_finalPose;
	}

	void beforeUpdate(double dt)
	{
		m_previousPose = m_currentPose;
	}

	void update(double dt)
	{
		m_currentPose.translation() += SurgSim::Math::Vector3d(1.0, 2.0, 3.0);
	}

	void afterUpdate(double dt)
	{
		m_finalPose = m_currentPose;
	}

	void resetState(void)
	{
		m_currentPose  = m_initialPose;
		m_previousPose = m_initialPose;
		m_finalPose    = m_initialPose;
	}

private:

	/// Pose of the actor
	RigidTransform3d m_initialPose;

	/// Pose of the actor
	RigidTransform3d m_previousPose;

	/// Pose of the actor
	RigidTransform3d m_currentPose;

	/// Pose of the actor
	RigidTransform3d m_finalPose;
};

TEST(ActorTest, ConstructorTest)
{
	ASSERT_NO_THROW({MockActor actor("Actor");});
}

TEST(ActorTest, SetGetAndDefaultValueTest)
{
	/// Create the actor
	std::shared_ptr<Actor> actor = std::make_shared<MockActor>("Actor");

	/// Get/Set active flag [default = true]
	EXPECT_TRUE(actor->isActive());
	actor->setIsActive(false);
	ASSERT_FALSE(actor->isActive());
	actor->setIsActive(true);
	ASSERT_TRUE(actor->isActive());

	/// Get numDof = 0
	ASSERT_EQ(0u, actor->getNumDof());

	/// Set/Get isGravityEnabled [default = true]
	EXPECT_TRUE(actor->isGravityEnabled());
	actor->setIsGravityEnabled(false);
	ASSERT_FALSE(actor->isGravityEnabled());
	actor->setIsGravityEnabled(true);
	ASSERT_TRUE(actor->isGravityEnabled());
}

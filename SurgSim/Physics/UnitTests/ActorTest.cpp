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

	/// Sets the pose of the actor
	virtual void setPose(const RigidTransform3d& transform)
	{
		m_pose = transform;
	}
	/// Returns the pose of the actor
	virtual const RigidTransform3d& getPose() const
	{
		return m_pose;
	}
private:
	/// Pose of the actor
	RigidTransform3d m_pose;
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

	/// Create a rigid body transform
	Vector3d translation = Vector3d::Random();
	Quaterniond rotation = Quaterniond(SurgSim::Math::Vector4d::Random());
	RigidTransform3d transform = SurgSim::Math::makeRigidTransform(rotation, translation);

	/// Set the pose and make sure it was set correctly
	actor->setPose(transform);
	EXPECT_TRUE(actor->getPose().isApprox(transform));
}

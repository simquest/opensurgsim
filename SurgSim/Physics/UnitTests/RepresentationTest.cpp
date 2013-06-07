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

#include <SurgSim/Physics/Representation.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

using SurgSim::Physics::Representation;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

/// Concrete representation class for testing
class MockRepresentation : public Representation
{
public:
	/// Constructor
	/// \name	Name of the representation
	MockRepresentation(const std::string& name) : Representation(name)
	{
	}

	/// Sets the pose of the representation
	virtual void setPose(const RigidTransform3d& transform)
	{
		m_pose = transform;
	}
	/// Returns the pose of the representation
	virtual const RigidTransform3d& getPose() const
	{
		return m_pose;
	}
private:
	/// Pose of the representation
	RigidTransform3d m_pose;
};

TEST(RepresentationTest, ConstructorTest)
{
	ASSERT_NO_THROW({MockRepresentation representation("Representation");});
}

TEST(RepresentationTest, SetGetAndDefaultValueTest)
{
	/// Create the representation
	std::shared_ptr<Representation> representation = std::make_shared<MockRepresentation>("Representation");

	/// Get/Set active flag [default = true]
	EXPECT_TRUE(representation->isActive());
	representation->setIsActive(false);
	ASSERT_FALSE(representation->isActive());
	representation->setIsActive(true);
	ASSERT_TRUE(representation->isActive());

	/// Get numDof = 0
	ASSERT_EQ(0u, representation->getNumDof());

	/// Set/Get isGravityEnabled [default = true]
	EXPECT_TRUE(representation->isGravityEnabled());
	representation->setIsGravityEnabled(false);
	ASSERT_FALSE(representation->isGravityEnabled());
	representation->setIsGravityEnabled(true);
	ASSERT_TRUE(representation->isGravityEnabled());

	/// Create a rigid body transform
	Vector3d translation = Vector3d::Random();
	Quaterniond rotation = Quaterniond(SurgSim::Math::Vector4d::Random());
	RigidTransform3d transform = SurgSim::Math::makeRigidTransform(rotation, translation);

	/// Set the pose and make sure it was set correctly
	representation->setPose(transform);
	EXPECT_TRUE(representation->getPose().isApprox(transform));
}

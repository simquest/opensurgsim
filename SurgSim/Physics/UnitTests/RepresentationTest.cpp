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
	explicit MockRepresentation(const std::string& name) : Representation(name)
	{
	}

	/// Sets the initial pose
	/// \param pose The initial pose to set the MockRepresentation to
	void setInitialPose(const RigidTransform3d& pose)
	{
		m_initialPose = pose;
	}

	/// Gets the initial pose of the representation
	/// \return The initial pose of this MockRepresentation
	const RigidTransform3d& getInitialPose() const
	{
		return m_initialPose;
	}

	/// Sets the current pose of the representation
	/// \param pose The current pose of this MockRepresentation
	void setPose(const RigidTransform3d& pose)
	{
		m_currentPose = pose;
	}

	/// Returns the previous pose of the representation
	/// \return The previous pose of this MockRepresentation
	const RigidTransform3d& getPreviousPose() const
	{
		return m_previousPose;
	}

	/// Returns the final pose of the representation
	/// \return The final pose of this MockRepresentation
	const RigidTransform3d& getPose() const
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
	/// Pose of the representation
	RigidTransform3d m_initialPose;

	/// Pose of the representation
	RigidTransform3d m_previousPose;

	/// Pose of the representation
	RigidTransform3d m_currentPose;

	/// Pose of the representation
	RigidTransform3d m_finalPose;
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
}

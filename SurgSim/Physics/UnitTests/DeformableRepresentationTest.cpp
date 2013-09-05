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

#include <SurgSim/Physics/DeformableRepresentation.h>
#include <SurgSim/Math/Vector.h>

namespace
{
	const unsigned int numDof = 154;
	const double epsilon = 1e-10;
};

class MockObject : public SurgSim::Physics::DeformableRepresentation
{
public:
	MockObject()
		: DeformableRepresentation("MockObject")
	{
		setNumDof(numDof);
	}

	virtual ~MockObject()
	{
	}

	/// Query the representation type
	/// \return the RepresentationType for this representation
	/// \note DeformableRepresentation is abstract because there is really no deformable behind this class !
	/// \note For the test, we simply set the type to INVALID
	virtual SurgSim::Physics::RepresentationType getType() const override
	{
		return SurgSim::Physics::REPRESENTATION_TYPE_INVALID;
	}

	/// before update method
	/// \param dt The time step for the current update
	virtual void beforeUpdate(double dt) override
	{
		// Backup the current state into the previous state
		m_previousState = m_currentState;
	}

	/// update method
	/// \param dt The time step for the current update
	virtual void update(double dt) override
	{
		// Update the current state with something... (+=)
		for(unsigned int i = 0; i < m_currentState.getNumDof(); i++)
		{
			m_currentState.getVelocities()[i] += static_cast<double>(i);
			m_currentState.getPositions()[i]  += m_currentState.getVelocities()[i] * dt;
		}
	}

	/// after update method
	/// \param dt The time step for the current update
	virtual void afterUpdate(double dt) override
	{
		// Backup the current state into the final state
		m_finalState = m_currentState;
	}

	void setInitialState(const SurgSim::Physics::DeformableRepresentationState& initialState)
	{
		m_initialState  = initialState;

		m_currentState  = initialState;
		m_previousState = initialState;
		m_finalState    = initialState;
	}
};

class DeformableRepresentationTest: public MockObject, public ::testing::Test
{
public:
	DeformableRepresentationTest()
		: MockObject()
	{
	}

	virtual ~DeformableRepresentationTest()
	{
	}

	/// Setup the test case
	void SetUp() override
	{
		m_localInitialState.allocate(numDof);
		for (unsigned int i = 0; i < numDof; i++)
		{
			m_localInitialState.getPositions()[i] = static_cast<double>(i);
			m_localInitialState.getVelocities()[i] = 1.0;
		}

		SurgSim::Math::Quaterniond q(0.1, 0.4, 0.5, 0.2);
		q.normalize();
		SurgSim::Math::Vector3d t(1.45, 5.4, 2.42);
		m_nonIdentityTransform = SurgSim::Math::makeRigidTransform(q, t);
		m_identityTransform = SurgSim::Math::RigidTransform3d::Identity();
	}

protected:
	// Initial state
	SurgSim::Physics::DeformableRepresentationState m_localInitialState;

	// Identity and nonIdentity (but still valid) transforms
	SurgSim::Math::RigidTransform3d m_identityTransform;
	SurgSim::Math::RigidTransform3d m_nonIdentityTransform;
};

TEST_F(DeformableRepresentationTest, ConstructorTest)
{
	// Test the constructor normally
	ASSERT_NO_THROW({MockObject deformable;});

	// Test the object creation through the operator new
	ASSERT_NO_THROW({MockObject *deformable = new MockObject; delete deformable;});

	// Test the object creation through the operator new []
	ASSERT_NO_THROW({MockObject *deformable = new MockObject[10]; delete [] deformable;});
}

TEST_F(DeformableRepresentationTest, SetGetTest)
{
		// Test getNumDof
	EXPECT_EQ(numDof, getNumDof());

	// Test setInitialPose/getInitialPose
	setInitialPose(m_nonIdentityTransform);
	EXPECT_TRUE(getInitialPose().isApprox(m_nonIdentityTransform, epsilon));
	EXPECT_FALSE(getInitialPose().isApprox(m_identityTransform, epsilon));
	setInitialPose(m_identityTransform);
	EXPECT_FALSE(getInitialPose().isApprox(m_nonIdentityTransform, epsilon));
	EXPECT_TRUE(getInitialPose().isApprox(m_identityTransform, epsilon));

	// Test setPose/getPose (always return Identity)
	EXPECT_THROW(setPose(m_nonIdentityTransform), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(setPose(m_identityTransform), SurgSim::Framework::AssertionFailure);
	EXPECT_TRUE(getPose().isApprox(m_identityTransform, epsilon));

	// Test setInitialState/getInitialState/getCurrentState
	EXPECT_EQ(getNumDof(), m_localInitialState.getNumDof());
	setInitialState(m_localInitialState);
	EXPECT_TRUE(m_initialState    == m_localInitialState);
	EXPECT_TRUE(m_currentState    == m_localInitialState);
	EXPECT_TRUE(m_previousState   == m_localInitialState);
	EXPECT_TRUE(m_finalState      == m_localInitialState);
	EXPECT_TRUE(getInitialState() == m_localInitialState);
	EXPECT_TRUE(getCurrentState() == m_localInitialState);
}

TEST_F(DeformableRepresentationTest, UpdateChangesStateTest)
{
	// setInitialState sets all 4 states (tested in method above !)
	setInitialState(m_localInitialState);

	// beforeUpdate should backup current (=initial) into previous
	beforeUpdate(1e-3);
	// update should change current
	update(1e-3);
	// afterUpdate should backup current into final
	afterUpdate(1e-3);
	EXPECT_TRUE (m_localInitialState == m_initialState);
	EXPECT_TRUE (m_localInitialState == m_previousState);
	EXPECT_FALSE(m_localInitialState == m_currentState);
	EXPECT_FALSE(m_localInitialState == m_finalState);
	EXPECT_FALSE(m_previousState     == m_currentState);
	EXPECT_TRUE (m_localInitialState == getInitialState());
	EXPECT_FALSE(m_localInitialState == getCurrentState());

	// beforeUpdate should backup current (!=initial) into previous
	beforeUpdate(1e-3);
	// update should change current
	update(1e-3);
	// afterUpdate should backup current into final
	afterUpdate(1e-3);
	EXPECT_TRUE (m_localInitialState == m_initialState);
	EXPECT_FALSE(m_localInitialState == m_previousState);
	EXPECT_FALSE(m_localInitialState == m_currentState);
	EXPECT_FALSE(m_localInitialState == m_finalState);
	EXPECT_FALSE(m_previousState     == m_currentState);
	EXPECT_TRUE (m_localInitialState == getInitialState());
	EXPECT_FALSE(m_localInitialState == getCurrentState());
}

TEST_F(DeformableRepresentationTest, ResetTest)
{
	// setInitialState sets all 4 states (tested in method above !)
	setInitialState(m_localInitialState);

	// 1st time step
	beforeUpdate(1e-3);
	update(1e-3);
	afterUpdate(1e-3);

	// 2nd time step
	beforeUpdate(1e-3);
	update(1e-3);
	afterUpdate(1e-3);

	EXPECT_TRUE (m_localInitialState == m_initialState);
	EXPECT_FALSE(m_localInitialState == m_previousState);
	EXPECT_FALSE(m_localInitialState == m_currentState);
	EXPECT_FALSE(m_localInitialState == m_finalState);
	EXPECT_TRUE (m_localInitialState == getInitialState());
	EXPECT_FALSE(m_localInitialState == getCurrentState());
	resetState();
	// reset should re-initialized current, previous and final to initial
	EXPECT_TRUE(m_localInitialState == m_initialState);
	EXPECT_TRUE(m_localInitialState == m_previousState);
	EXPECT_TRUE(m_localInitialState == m_currentState);
	EXPECT_TRUE(m_localInitialState == m_finalState);
	EXPECT_TRUE(m_localInitialState == getInitialState());
	EXPECT_TRUE(m_localInitialState == getCurrentState());
}

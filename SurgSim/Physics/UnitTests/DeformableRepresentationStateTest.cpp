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

#include <SurgSim/Physics/DeformableRepresentationState.h>

TEST(DeformableActorStateTest, ConstructorTest)
{
	using SurgSim::Physics::DeformableRepresentationState;

	// Test the constructor normally
	ASSERT_NO_THROW( {DeformableRepresentationState state;});

	// Test the object creation through the operator new
	// Eigen needs special care with fixed-size matrix member variables of a class created dynamically via new.
	// We are using non fixed-size matrix, so it should be all fine...this is just to make sure.
	ASSERT_NO_THROW( {DeformableRepresentationState* state = new DeformableRepresentationState; delete state; });

	// Test the object creation through the operator new []
	// Eigen needs special care with fixed-size matrix member variables of a class created dynamically via new [].
	// We are using non fixed-size matrix, so it should be all fine...this is just to make sure.
	ASSERT_NO_THROW( {DeformableRepresentationState* state = new DeformableRepresentationState[10]; delete [] state; });
}

TEST(DeformableActorStateTest, AllocateTest)
{
	using SurgSim::Physics::DeformableRepresentationState;
	
	DeformableRepresentationState state;
	ASSERT_NO_THROW(state.allocate(10));
	EXPECT_EQ(10, state.getPositions().size());
}

TEST(DeformableActorStateTest, GetPositionsTest)
{
	using SurgSim::Physics::DeformableRepresentationState;
	
	DeformableRepresentationState state1, state2;
	state1.allocate(10);
	state2.allocate(10);
	for(int i = 0; i < state1.getPositions().size(); i++)
	{
		state1.getPositions()[i] = (double)i;
		state2.getPositions()[i] = 0.0;
	}
	// state1.m_x contains (0 1 2 3 4 5 6 7 8 9 10) & state2.m_x contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_NE(state2.getPositions(), state1.getPositions());
	state2.getPositions() = state1.getPositions();
	// state1.m_x contains (0 1 2 3 4 5 6 7 8 9 10) & state2.m_x contains (0 1 2 3 4 5 6 7 8 9 10)
	EXPECT_EQ(state2.getPositions(), state1.getPositions());

	state1.reset();
	// state1.m_x contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_x contains (0 1 2 3 4 5 6 7 8 9 10)
	for(int i = 0; i < state1.getPositions().size(); i++)
	{
		EXPECT_EQ(0.0, state1.getPositions()[i]);
		EXPECT_EQ((double)i, state2.getPositions()[i]);
	}

	state2.reset();
	// state1.m_x contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_x contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_EQ(state2.getPositions(), state1.getPositions());
}

TEST(DeformableActorStateTest, GetVelocitiesTest)
{
	using SurgSim::Physics::DeformableRepresentationState;
	
	DeformableRepresentationState state1, state2;
	state1.allocate(10);
	state2.allocate(10);
	for(int i = 0; i < state1.getVelocities().size(); i++)
	{
		state1.getVelocities()[i] = (double)i;
		state2.getVelocities()[i] = 0.0;
	}
	// state1.m_x contains (0 1 2 3 4 5 6 7 8 9 10) & state2.m_x contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_NE(state2.getVelocities(), state1.getVelocities());
	state2.getVelocities() = state1.getVelocities();
	// state1.m_x contains (0 1 2 3 4 5 6 7 8 9 10) & state2.m_x contains (0 1 2 3 4 5 6 7 8 9 10)
	EXPECT_EQ(state2.getVelocities(), state1.getVelocities());

	state1.reset();
	// state1.m_x contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_x contains (0 1 2 3 4 5 6 7 8 9 10)
	for(int i = 0; i < state1.getVelocities().size(); i++)
	{
		EXPECT_EQ(0.0, state1.getVelocities()[i]);
		EXPECT_EQ((double)i, state2.getVelocities()[i]);
	}

	state2.reset();
	// state1.m_x contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_x contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_EQ(state2.getVelocities(), state1.getVelocities());
}

TEST(DeformableActorStateTest, ResetTest)
{
	using SurgSim::Physics::DeformableRepresentationState;
	
	DeformableRepresentationState state1, state2;
	state1.allocate(10);
	state2.allocate(10);
	state2.reset();
	for(int i = 0; i < state1.getPositions().size(); i++)
	{
		state1.getPositions()[i] = (double)i;
		state1.getVelocities()[i] = 2.0*(double)i;
	}
	EXPECT_NE(state2, state1);
	state1.reset();
	EXPECT_EQ(state2, state1);
	for(int i = 0; i < state1.getPositions().size(); i++)
	{
		EXPECT_EQ(0.0, state1.getPositions()[i]);
		EXPECT_EQ(0.0, state1.getVelocities()[i]);
	}
}

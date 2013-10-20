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

#include <memory>

#include <gtest/gtest.h>

#include <SurgSim/Physics/DeformableRepresentationState.h>
using SurgSim::Physics::DeformableRepresentationState;

namespace
{
	const double epsilon = 1e-10;
};

TEST(DeformableRepresentationStateTest, ConstructorTest)
{
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

	// Test the object creation through std::shared_ptr
	// Eigen needs special care with fixed-size matrix member variables of a class created dynamically via new.
	// We are using non fixed-size matrix, so it should be all fine...this is just to make sure.
	ASSERT_NO_THROW( {std::shared_ptr<DeformableRepresentationState> state = \
		std::make_shared<DeformableRepresentationState>();});
}

TEST(DeformableRepresentationStateTest, AllocateTest)
{
	DeformableRepresentationState state;
	EXPECT_EQ(0u, state.getNumDof());
	EXPECT_EQ(0u, state.getNumNodes());
	EXPECT_EQ(0u, state.getNumBoundaryConditions());
	EXPECT_EQ(0u, state.getBoundaryConditions().size());
	EXPECT_EQ(0u, state.getPositions().size());
	EXPECT_EQ(0u, state.getVelocities().size());
	EXPECT_EQ(0u, state.getAccelerations().size());

	ASSERT_NO_THROW(state.setNumDof(3u, 3u));
	EXPECT_EQ(9u, state.getNumDof());
	EXPECT_EQ(3u, state.getNumNodes());
	EXPECT_EQ(9u, state.getPositions().size());
	EXPECT_EQ(9u, state.getVelocities().size());
	EXPECT_EQ(9u, state.getAccelerations().size());
	EXPECT_EQ(0u , state.getNumBoundaryConditions());
	EXPECT_EQ(0u , state.getBoundaryConditions().size());
}

TEST(DeformableRepresentationStateTest, GetPositionsTest)
{
	DeformableRepresentationState state1, state2;
	state1.setNumDof(3u, 3u);
	state2.setNumDof(3u, 3u);
	for(unsigned int i = 0; i < state1.getNumDof(); i++)
	{
		state1.getPositions()[i] = static_cast<double>(i);
		state2.getPositions()[i] = 0.0;
	}
	// state1.m_x contains (0 1 2 3 4 5 6 7 8 9 10) & state2.m_x contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_NE(state2.getPositions(), state1.getPositions());
	state2.getPositions() = state1.getPositions();
	// state1.m_x contains (0 1 2 3 4 5 6 7 8 9 10) & state2.m_x contains (0 1 2 3 4 5 6 7 8 9 10)
	EXPECT_EQ(state2.getPositions(), state1.getPositions());

	state1.reset();
	// state1.m_x contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_x contains (0 1 2 3 4 5 6 7 8 9 10)
	for(unsigned int i = 0; i < state1.getNumDof(); i++)
	{
		EXPECT_EQ(0.0, state1.getPositions()[i]);
		EXPECT_EQ(static_cast<double>(i), state2.getPositions()[i]);
	}

	state2.reset();
	// state1.m_x contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_x contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_EQ(state2.getPositions(), state1.getPositions());
}

TEST(DeformableRepresentationStateTest, GetVelocitiesTest)
{
	DeformableRepresentationState state1, state2;
	state1.setNumDof(3u, 3u);
	state2.setNumDof(3u, 3u);
	for(unsigned int i = 0; i < state1.getNumDof(); i++)
	{
		state1.getVelocities()[i] = static_cast<double>(i);
		state2.getVelocities()[i] = 0.0;
	}
	// state1.m_v contains (0 1 2 3 4 5 6 7 8 9 10) & state2.m_v contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_NE(state2.getVelocities(), state1.getVelocities());
	state2.getVelocities() = state1.getVelocities();
	// state1.m_v contains (0 1 2 3 4 5 6 7 8 9 10) & state2.m_v contains (0 1 2 3 4 5 6 7 8 9 10)
	EXPECT_EQ(state2.getVelocities(), state1.getVelocities());

	state1.reset();
	// state1.m_v contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_v contains (0 1 2 3 4 5 6 7 8 9 10)
	for(unsigned int i = 0; i < state1.getNumDof(); i++)
	{
		EXPECT_EQ(0.0, state1.getVelocities()[i]);
		EXPECT_EQ(static_cast<double>(i), state2.getVelocities()[i]);
	}

	state2.reset();
	// state1.m_v contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_v contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_EQ(state2.getVelocities(), state1.getVelocities());
}

TEST(DeformableRepresentationStateTest, GetAccelerationsTest)
{
	DeformableRepresentationState state1, state2;
	state1.setNumDof(3u, 3u);
	state2.setNumDof(3u, 3u);
	for(unsigned int i = 0; i < state1.getNumDof(); i++)
	{
		state1.getAccelerations()[i] = static_cast<double>(i);
		state2.getAccelerations()[i] = 0.0;
	}
	// state1.m_a contains (0 1 2 3 4 5 6 7 8 9 10) & state2.m_a contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_NE(state2.getAccelerations(), state1.getAccelerations());
	state2.getAccelerations() = state1.getAccelerations();
	// state1.m_a contains (0 1 2 3 4 5 6 7 8 9 10) & state2.m_a contains (0 1 2 3 4 5 6 7 8 9 10)
	EXPECT_EQ(state2.getAccelerations(), state1.getAccelerations());

	state1.reset();
	// state1.m_a contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_a contains (0 1 2 3 4 5 6 7 8 9 10)
	for(unsigned int i = 0; i < state1.getNumDof(); i++)
	{
		EXPECT_EQ(0.0, state1.getAccelerations()[i]);
		EXPECT_EQ(static_cast<double>(i), state2.getAccelerations()[i]);
	}

	state2.reset();
	// state1.m_a contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_a contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_EQ(state2.getAccelerations(), state1.getAccelerations());
}

TEST(DeformableRepresentationStateTest, AddGetIsBoundaryConditionsTest)
{
	DeformableRepresentationState state;
	state.setNumDof(3u, 2u);

	state.addBoundaryCondition(0);
	EXPECT_EQ(6u, state.getNumDof());
	EXPECT_EQ(1u, state.getNumBoundaryConditions());
	ASSERT_EQ(1u, state.getBoundaryConditions().size());
	for (unsigned int dofId = 1; dofId < 6; dofId++)
	{
		if (dofId == 0u)
		{
			EXPECT_TRUE(state.isBoundaryCondition(dofId));
		}
		else
		{
			EXPECT_FALSE(state.isBoundaryCondition(dofId));
		}
	}
	EXPECT_EQ(0u, state.getBoundaryConditions()[0]);
	EXPECT_EQ(6u, state.getPositions().size());
	EXPECT_EQ(6u, state.getVelocities().size());
	EXPECT_EQ(6u, state.getAccelerations().size());
	EXPECT_TRUE(state.getPositions().isZero());
	EXPECT_TRUE(state.getVelocities().isZero());
	EXPECT_TRUE(state.getAccelerations().isZero());

	state.addBoundaryCondition(2);
	EXPECT_EQ(6u, state.getNumDof());
	EXPECT_EQ(2u, state.getNumBoundaryConditions());
	ASSERT_EQ(2u, state.getBoundaryConditions().size());
	EXPECT_EQ(0u, state.getBoundaryConditions()[0]);
	EXPECT_EQ(2u, state.getBoundaryConditions()[1]);
	for (unsigned int dofId = 1; dofId < 6; dofId++)
	{
		if (dofId == 0u || dofId == 2u)
		{
			EXPECT_TRUE(state.isBoundaryCondition(dofId));
		}
		else
		{
			EXPECT_FALSE(state.isBoundaryCondition(dofId));
		}
	}
	EXPECT_EQ(6u, state.getPositions().size());
	EXPECT_EQ(6u, state.getVelocities().size());
	EXPECT_EQ(6u, state.getAccelerations().size());
	EXPECT_TRUE(state.getPositions().isZero());
	EXPECT_TRUE(state.getVelocities().isZero());
	EXPECT_TRUE(state.getAccelerations().isZero());

	state.addBoundaryCondition(4);
	EXPECT_EQ(6u, state.getNumDof());
	EXPECT_EQ(3u, state.getNumBoundaryConditions());
	ASSERT_EQ(3u, state.getBoundaryConditions().size());
	EXPECT_EQ(0u, state.getBoundaryConditions()[0]);
	EXPECT_EQ(2u, state.getBoundaryConditions()[1]);
	EXPECT_EQ(4u, state.getBoundaryConditions()[2]);
	for (unsigned int dofId = 1; dofId < 6; dofId++)
	{
		if (dofId == 0u || dofId == 2u || dofId == 4u)
		{
			EXPECT_TRUE(state.isBoundaryCondition(dofId));
		}
		else
		{
			EXPECT_FALSE(state.isBoundaryCondition(dofId));
		}
	}
	EXPECT_EQ(6u, state.getPositions().size());
	EXPECT_EQ(6u, state.getVelocities().size());
	EXPECT_EQ(6u, state.getAccelerations().size());
	EXPECT_TRUE(state.getPositions().isZero());
	EXPECT_TRUE(state.getVelocities().isZero());
	EXPECT_TRUE(state.getAccelerations().isZero());
}

TEST(DeformableRepresentationStateTest, ResetTest)
{
	DeformableRepresentationState state1, state2;
	state1.setNumDof(3u, 3u);
	state2.setNumDof(3u, 3u);
	for(unsigned int i = 0; i < state1.getNumDof(); i++)
	{
		state1.getPositions()[i] = static_cast<double>(i);
		state1.getVelocities()[i] = 2.0*static_cast<double>(i);
		state1.getAccelerations()[i] = 4.0*static_cast<double>(i);
	}
	state1.addBoundaryCondition(0);
	state1.addBoundaryCondition(state1.getNumDof() - 1);
	EXPECT_NE(state2, state1);

	state1.reset();
	EXPECT_EQ(state2, state1);
	EXPECT_EQ(9u, state1.getNumDof());
	EXPECT_EQ(3u, state1.getNumNodes());
	EXPECT_TRUE(state1.getPositions().isZero());
	EXPECT_TRUE(state1.getVelocities().isZero());
	EXPECT_TRUE(state1.getAccelerations().isZero());
	EXPECT_EQ(0u, state1.getNumBoundaryConditions());
	EXPECT_EQ(0u, state1.getBoundaryConditions().size());
}

TEST(DeformableRepresentationStateTest, CopyConstructorAndAssignmentTest)
{
	DeformableRepresentationState state, stateAssigned;
	state.setNumDof(3u, 3u);
	for(unsigned int i = 0; i < state.getNumDof(); i++)
	{
		state.getPositions()[i] = static_cast<double>(i);
		state.getVelocities()[i] = 2.0*static_cast<double>(i);
		state.getAccelerations()[i] = 4.0*static_cast<double>(i);
	}
	state.addBoundaryCondition(0);
	state.addBoundaryCondition(state.getNumDof() - 1);

	{
		DeformableRepresentationState stateCopied(state);

		ASSERT_EQ(9u, stateCopied.getNumDof());
		ASSERT_EQ(state.getNumDof(), stateCopied.getNumDof());
		ASSERT_EQ(9, stateCopied.getPositions().size());
		ASSERT_EQ(state.getPositions().size(), stateCopied.getPositions().size());
		ASSERT_EQ(9, stateCopied.getVelocities().size());
		ASSERT_EQ(state.getVelocities().size(), stateCopied.getVelocities().size());
		ASSERT_EQ(9, stateCopied.getAccelerations().size());
		ASSERT_EQ(state.getAccelerations().size(), stateCopied.getAccelerations().size());

		for(unsigned int i = 0; i < stateCopied.getNumDof(); i++)
		{
			EXPECT_NEAR(state.getPositions()[i], stateCopied.getPositions()[i], epsilon);
			EXPECT_NEAR(static_cast<double>(i), stateCopied.getPositions()[i], epsilon);
			EXPECT_NEAR(state.getVelocities()[i], stateCopied.getVelocities()[i], epsilon);
			EXPECT_NEAR(2.0*static_cast<double>(i), stateCopied.getVelocities()[i], epsilon);
			EXPECT_NEAR(state.getAccelerations()[i], stateCopied.getAccelerations()[i], epsilon);
			EXPECT_NEAR(4.0*static_cast<double>(i), stateCopied.getAccelerations()[i], epsilon);
		}

		ASSERT_EQ(2u, stateCopied.getNumBoundaryConditions());
		ASSERT_EQ(state.getNumBoundaryConditions(), stateCopied.getNumBoundaryConditions());
		ASSERT_EQ(2u, stateCopied.getBoundaryConditions().size());
		ASSERT_EQ(state.getBoundaryConditions().size(), stateCopied.getBoundaryConditions().size());
		ASSERT_EQ(0u, stateCopied.getBoundaryConditions()[0]);
		ASSERT_EQ(state.getBoundaryConditions()[0], stateCopied.getBoundaryConditions()[0]);
		ASSERT_EQ(state.getNumDof() - 1, stateCopied.getBoundaryConditions()[1]);
		ASSERT_EQ(state.getBoundaryConditions()[1], stateCopied.getBoundaryConditions()[1]);
	}

	{
		stateAssigned = state;

		ASSERT_EQ(9u, stateAssigned.getNumDof());
		ASSERT_EQ(state.getNumDof(), stateAssigned.getNumDof());
		ASSERT_EQ(9, stateAssigned.getPositions().size());
		ASSERT_EQ(state.getPositions().size(), stateAssigned.getPositions().size());
		ASSERT_EQ(9, stateAssigned.getVelocities().size());
		ASSERT_EQ(state.getVelocities().size(), stateAssigned.getVelocities().size());
		ASSERT_EQ(9, stateAssigned.getAccelerations().size());
		ASSERT_EQ(state.getAccelerations().size(), stateAssigned.getAccelerations().size());

		for(unsigned int i = 0; i < stateAssigned.getNumDof(); i++)
		{
			EXPECT_NEAR(state.getPositions()[i], stateAssigned.getPositions()[i], epsilon);
			EXPECT_NEAR(static_cast<double>(i), stateAssigned.getPositions()[i], epsilon);
			EXPECT_NEAR(state.getVelocities()[i], stateAssigned.getVelocities()[i], epsilon);
			EXPECT_NEAR(2.0*static_cast<double>(i), stateAssigned.getVelocities()[i], epsilon);
			EXPECT_NEAR(state.getAccelerations()[i], stateAssigned.getAccelerations()[i], epsilon);
			EXPECT_NEAR(4.0*static_cast<double>(i), stateAssigned.getAccelerations()[i], epsilon);
		}

		ASSERT_EQ(2u, stateAssigned.getNumBoundaryConditions());
		ASSERT_EQ(state.getNumBoundaryConditions(), stateAssigned.getNumBoundaryConditions());
		ASSERT_EQ(2u, stateAssigned.getBoundaryConditions().size());
		ASSERT_EQ(state.getBoundaryConditions().size(), stateAssigned.getBoundaryConditions().size());
		ASSERT_EQ(0u, stateAssigned.getBoundaryConditions()[0]);
		ASSERT_EQ(state.getBoundaryConditions()[0], stateAssigned.getBoundaryConditions()[0]);
		ASSERT_EQ(state.getNumDof() - 1, stateAssigned.getBoundaryConditions()[1]);
		ASSERT_EQ(state.getBoundaryConditions()[1], stateAssigned.getBoundaryConditions()[1]);
	}
}

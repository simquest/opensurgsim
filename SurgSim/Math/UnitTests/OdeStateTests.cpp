// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Framework/Assert.h"

using SurgSim::Math::OdeState;

namespace
{
const double epsilon = 1e-10;
};

TEST(OdeStateTest, ConstructorTest)
{
	// Test the constructor normally
	ASSERT_NO_THROW({OdeState state;});

	// Test the object creation through the operator new
	// Eigen needs special care with fixed-size matrix member variables of a class created dynamically via new.
	// We are using non fixed-size matrix, so it should be all fine...this is just to make sure.
	ASSERT_NO_THROW({OdeState* state = new OdeState; delete state; });

	// Test the object creation through the operator new []
	// Eigen needs special care with fixed-size matrix member variables of a class created dynamically via new [].
	// We are using non fixed-size matrix, so it should be all fine...this is just to make sure.
	ASSERT_NO_THROW({OdeState* state = new OdeState[10]; delete [] state; });

	// Test the object creation through std::shared_ptr
	// Eigen needs special care with fixed-size matrix member variables of a class created dynamically via new.
	// We are using non fixed-size matrix, so it should be all fine...this is just to make sure.
	ASSERT_NO_THROW({std::shared_ptr<OdeState> state = std::make_shared<OdeState>();});
}

TEST(OdeStateTest, AllocateTest)
{
	OdeState state;
	EXPECT_EQ(0u, state.getNumDof());
	EXPECT_EQ(0u, state.getNumNodes());
	EXPECT_EQ(0u, state.getNumBoundaryConditions());
	EXPECT_EQ(0, state.getBoundaryConditions().size());
	EXPECT_EQ(0, state.getPositions().size());
	EXPECT_EQ(0, state.getVelocities().size());

	ASSERT_NO_THROW(state.setNumDof(3u, 3u));
	EXPECT_EQ(9u, state.getNumDof());
	EXPECT_EQ(3u, state.getNumNodes());
	EXPECT_EQ(9, state.getPositions().size());
	EXPECT_EQ(9, state.getVelocities().size());
	EXPECT_EQ(0u , state.getNumBoundaryConditions());
	EXPECT_EQ(0 , state.getBoundaryConditions().size());
}

TEST(OdeStateTest, GetPositionsTest)
{
	OdeState state1, state2;
	state1.setNumDof(3u, 3u);
	state2.setNumDof(3u, 3u);
	for (size_t i = 0; i < state1.getNumDof(); i++)
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
	for (size_t i = 0; i < state1.getNumDof(); i++)
	{
		EXPECT_EQ(0.0, state1.getPositions()[i]);
		EXPECT_EQ(static_cast<double>(i), state2.getPositions()[i]);
	}

	state2.reset();
	// state1.m_x contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_x contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_EQ(state2.getPositions(), state1.getPositions());
}

TEST(OdeStateTest, GetVelocitiesTest)
{
	OdeState state1, state2;
	state1.setNumDof(3u, 3u);
	state2.setNumDof(3u, 3u);
	for (size_t i = 0; i < state1.getNumDof(); i++)
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
	for (size_t i = 0; i < state1.getNumDof(); i++)
	{
		EXPECT_EQ(0.0, state1.getVelocities()[i]);
		EXPECT_EQ(static_cast<double>(i), state2.getVelocities()[i]);
	}

	state2.reset();
	// state1.m_v contains (0 0 0 0 0 0 0 0 0 0 0) & state2.m_v contains (0 0 0 0 0 0 0 0 0 0 0)
	EXPECT_EQ(state2.getVelocities(), state1.getVelocities());
}

namespace
{
void testBoundaryConditions(const SurgSim::Math::OdeState& state, std::vector<size_t> expectedDofIds)
{
	EXPECT_EQ(6u, state.getNumDof());
	EXPECT_EQ(expectedDofIds.size(), state.getNumBoundaryConditions());
	ASSERT_EQ(expectedDofIds.size(), state.getBoundaryConditions().size());
	for (size_t index = 0; index < expectedDofIds.size(); ++index)
	{
		EXPECT_EQ(expectedDofIds[index], state.getBoundaryConditions()[index]);
	}
	for (size_t dofId = 0; dofId < 6; dofId++)
	{
		if (std::find(expectedDofIds.begin(), expectedDofIds.end(), dofId) != expectedDofIds.end())
		{
			EXPECT_TRUE(state.isBoundaryCondition(dofId));
		}
		else
		{
			EXPECT_FALSE(state.isBoundaryCondition(dofId));
		}
	}
	EXPECT_EQ(6, state.getPositions().size());
	EXPECT_EQ(6, state.getVelocities().size());
	EXPECT_TRUE(state.getPositions().isZero());
	EXPECT_TRUE(state.getVelocities().isZero());
}
void testBoundaryConditionsStaticDof(const SurgSim::Math::OdeState& state,
									 std::vector<std::pair<size_t, double>> expectedDofIds)
{
	EXPECT_EQ(6u, state.getNumDof());
	EXPECT_EQ(expectedDofIds.size(), state.getNumBoundaryConditionsStaticDof());
	ASSERT_EQ(expectedDofIds.size(), state.getBoundaryConditionsStaticDof().size());
	for (size_t index = 0; index < expectedDofIds.size(); ++index)
	{
		EXPECT_EQ(expectedDofIds[index].first, state.getBoundaryConditionsStaticDof()[index].first);
		EXPECT_EQ(expectedDofIds[index].second, state.getBoundaryConditionsStaticDof()[index].second);
	}
}
}; // anonymous namespace

TEST(OdeStateTest, AddGetIsBoundaryConditionsTest)
{
	{
		OdeState state;
		std::vector<size_t> expectedDofIdsBoundaryConditions;

		// Assert trying to add a boundary condition before setting the number of node and dof per node
		ASSERT_THROW(state.addBoundaryCondition(0u, 0u), SurgSim::Framework::AssertionFailure);

		state.setNumDof(3u, 2u); // Number of dof per node is 3

		SCOPED_TRACE("Testing addBoundaryCondition(size_t nodeId, size_t dofId)");

		state.addBoundaryCondition(0u, 0u);
		expectedDofIdsBoundaryConditions.push_back((0u * 3u + 0u)); // (node 0, dof 0)
		testBoundaryConditions(state, expectedDofIdsBoundaryConditions);

		state.addBoundaryCondition(1u, 2u);
		expectedDofIdsBoundaryConditions.push_back((1u * 3u + 2u)); // (node 1, dof 2)
		testBoundaryConditions(state, expectedDofIdsBoundaryConditions);

		state.addBoundaryCondition(0u, 2u);
		expectedDofIdsBoundaryConditions.push_back((0u * 3u + 2u)); // (node 0, dof 2)
		testBoundaryConditions(state, expectedDofIdsBoundaryConditions);

		// Assert on wrong nodeId
		ASSERT_THROW(state.addBoundaryCondition(3u, 0u), SurgSim::Framework::AssertionFailure);

		// Assert on wrong dofId
		ASSERT_THROW(state.addBoundaryCondition(0u, 4u), SurgSim::Framework::AssertionFailure);
	}

	{
		OdeState state;
		std::vector<size_t> expectedDofIdsBoundaryConditions;

		// Assert trying to add a boundary condition before setting the number of node and dof per node
		ASSERT_THROW(state.addBoundaryCondition(0u), SurgSim::Framework::AssertionFailure);

		state.setNumDof(3u, 2u); // Number of dof per node is 3

		SCOPED_TRACE("Testing addBoundaryCondition(size_t nodeId)");

		state.addBoundaryCondition(0u);
		expectedDofIdsBoundaryConditions.push_back((0u * 3u + 0u)); // (node 0, dof 0)
		expectedDofIdsBoundaryConditions.push_back((0u * 3u + 1u)); // (node 0, dof 1)
		expectedDofIdsBoundaryConditions.push_back((0u * 3u + 2u)); // (node 0, dof 2)
		testBoundaryConditions(state, expectedDofIdsBoundaryConditions);

		// Assert on wrong nodeId
		ASSERT_THROW(state.addBoundaryCondition(3u), SurgSim::Framework::AssertionFailure);

		state.addBoundaryCondition(1u);
		expectedDofIdsBoundaryConditions.push_back((1u * 3u + 0u)); // (node 0, dof 0)
		expectedDofIdsBoundaryConditions.push_back((1u * 3u + 1u)); // (node 0, dof 1)
		expectedDofIdsBoundaryConditions.push_back((1u * 3u + 2u)); // (node 0, dof 2)
		testBoundaryConditions(state, expectedDofIdsBoundaryConditions);
	}
}

TEST(OdeStateTest, AddGetSetBoundaryConditionsStaticDofTest)
{
	OdeState state;
	std::vector<std::pair<size_t, double>> expected;

	// Assert trying to add a boundary condition before setting the number of node and dof per node
	ASSERT_THROW(state.addBoundaryConditionStaticDof(0u, 0.1), SurgSim::Framework::AssertionFailure);

	state.setNumDof(3u, 2u); // Number of dof per node is 3

	SCOPED_TRACE("Testing addBoundaryConditionStaticDof(size_t nodeId, double value)");

	state.addBoundaryConditionStaticDof(0u, 0.1);
	expected.push_back(std::make_pair(0u, 0.1)); // (node 0, value 0.1)
	testBoundaryConditionsStaticDof(state, expected);

	state.addBoundaryConditionStaticDof(1u, 2.6);
	expected.push_back(std::make_pair(1u, 2.6)); // (node 1, value 2.6)
	testBoundaryConditionsStaticDof(state, expected);

	state.setBoundaryConditionStaticDof(0u, 5.4);
	expected.front().second = 5.4; // (node 0, value 5.4)
	testBoundaryConditionsStaticDof(state, expected);

	// Assert on wrong nodeId
	ASSERT_THROW(state.addBoundaryConditionStaticDof(3u, 0.0), SurgSim::Framework::AssertionFailure);
}

TEST(OdeStateTest, ResetTest)
{
	OdeState state1, state2;
	state1.setNumDof(3u, 3u);
	state2.setNumDof(3u, 3u);
	for (size_t i = 0; i < state1.getNumDof(); i++)
	{
		state1.getPositions()[i] = static_cast<double>(i);
		state1.getVelocities()[i] = 2.0 * static_cast<double>(i);
	}
	state1.addBoundaryCondition(0u, 0u);
	state1.addBoundaryCondition(state1.getNumNodes() - 1u, 2u);
	EXPECT_NE(state2, state1);

	state1.reset();
	EXPECT_EQ(state2, state1);
	EXPECT_EQ(9u, state1.getNumDof());
	EXPECT_EQ(3u, state1.getNumNodes());
	EXPECT_TRUE(state1.getPositions().isZero());
	EXPECT_TRUE(state1.getVelocities().isZero());
	EXPECT_EQ(0u, state1.getNumBoundaryConditions());
	EXPECT_EQ(0, state1.getBoundaryConditions().size());
}

TEST(OdeStateTest, CopyConstructorAndAssignmentTest)
{
	OdeState state, stateAssigned;
	state.setNumDof(3u, 3u);
	for (size_t i = 0; i < state.getNumDof(); i++)
	{
		state.getPositions()[i] = static_cast<double>(i);
		state.getVelocities()[i] = 2.0 * static_cast<double>(i);
	}
	state.addBoundaryCondition(0u, 0u);
	state.addBoundaryCondition(state.getNumNodes() - 1u, 2u);

	{
		OdeState stateCopied(state);

		ASSERT_EQ(9u, stateCopied.getNumDof());
		ASSERT_EQ(state.getNumDof(), stateCopied.getNumDof());
		ASSERT_EQ(9, stateCopied.getPositions().size());
		ASSERT_EQ(state.getPositions().size(), stateCopied.getPositions().size());
		ASSERT_EQ(9, stateCopied.getVelocities().size());
		ASSERT_EQ(state.getVelocities().size(), stateCopied.getVelocities().size());

		for (size_t i = 0; i < stateCopied.getNumDof(); i++)
		{
			EXPECT_NEAR(state.getPositions()[i], stateCopied.getPositions()[i], epsilon);
			EXPECT_NEAR(static_cast<double>(i), stateCopied.getPositions()[i], epsilon);
			EXPECT_NEAR(state.getVelocities()[i], stateCopied.getVelocities()[i], epsilon);
			EXPECT_NEAR(2.0 * static_cast<double>(i), stateCopied.getVelocities()[i], epsilon);
		}

		ASSERT_EQ(2u, stateCopied.getNumBoundaryConditions());
		ASSERT_EQ(state.getNumBoundaryConditions(), stateCopied.getNumBoundaryConditions());
		ASSERT_EQ(2, stateCopied.getBoundaryConditions().size());
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

		for (size_t i = 0; i < stateAssigned.getNumDof(); i++)
		{
			EXPECT_NEAR(state.getPositions()[i], stateAssigned.getPositions()[i], epsilon);
			EXPECT_NEAR(static_cast<double>(i), stateAssigned.getPositions()[i], epsilon);
			EXPECT_NEAR(state.getVelocities()[i], stateAssigned.getVelocities()[i], epsilon);
			EXPECT_NEAR(2.0 * static_cast<double>(i), stateAssigned.getVelocities()[i], epsilon);
		}

		ASSERT_EQ(2u, stateAssigned.getNumBoundaryConditions());
		ASSERT_EQ(state.getNumBoundaryConditions(), stateAssigned.getNumBoundaryConditions());
		ASSERT_EQ(2, stateAssigned.getBoundaryConditions().size());
		ASSERT_EQ(state.getBoundaryConditions().size(), stateAssigned.getBoundaryConditions().size());
		ASSERT_EQ(0u, stateAssigned.getBoundaryConditions()[0]);
		ASSERT_EQ(state.getBoundaryConditions()[0], stateAssigned.getBoundaryConditions()[0]);
		ASSERT_EQ(state.getNumDof() - 1, stateAssigned.getBoundaryConditions()[1]);
		ASSERT_EQ(state.getBoundaryConditions()[1], stateAssigned.getBoundaryConditions()[1]);
	}
}

TEST(OdeStateTest, ApplyBoundaryConditionsToVectorTest)
{
	OdeState state;
	state.setNumDof(3u, 3u);
	state.addBoundaryCondition(0u, 1u);
	state.addBoundaryCondition(state.getNumNodes() - 1u, 2u);

	SurgSim::Math::Vector F(state.getNumDof());
	F.setLinSpaced(1.0, 2.0);
	SurgSim::Math::Vector initialF = F;

	state.applyBoundaryConditionsToVector(&F);
	EXPECT_FALSE(F.isApprox(initialF));
	for (size_t dofId = 0; dofId < state.getNumDof(); ++dofId)
	{
		if (dofId == 1u || dofId == state.getNumDof() - 1u)
		{
			EXPECT_NE(initialF[dofId], F[dofId]);
			EXPECT_EQ(0u, F[dofId]);
		}
		else
		{
			EXPECT_EQ(initialF[dofId], F[dofId]);
		}
	}
}

TEST(OdeStateTest, ApplyBoundaryConditionsToMatrixTest)
{
	OdeState state;
	state.setNumDof(3u, 3u);
	state.addBoundaryCondition(0u, 1u);
	state.addBoundaryCondition(state.getNumNodes() - 1u, 2u);

	size_t numDof = state.getNumDof();
	SurgSim::Math::Matrix M = 2.0 * SurgSim::Math::Matrix::Ones(numDof, numDof);
	SurgSim::Math::Matrix initialM = M;
	SurgSim::Math::Matrix expectedM = M;
	for (int bcId = 0; bcId < 2; ++bcId)
	{
		size_t dofId = state.getBoundaryConditions()[bcId];
		expectedM.block<1, 9>(dofId, 0).setZero();
		expectedM.block<9, 1>(0, dofId).setZero();
		expectedM(dofId, dofId) = 1.0;
	}

	state.applyBoundaryConditionsToMatrix(&M);
	EXPECT_FALSE(M.isApprox(initialM));
	EXPECT_TRUE(M.isApprox(expectedM));
}

namespace
{
void testOdeStateValidityWith(double number, bool isValid)
{
	OdeState stateWithNumberOnPosition;
	stateWithNumberOnPosition.setNumDof(3u, 3u);
	stateWithNumberOnPosition.getPositions().setOnes();
	stateWithNumberOnPosition.getPositions()[2] = number;
	if (isValid)
	{
		EXPECT_TRUE(stateWithNumberOnPosition.isValid());
	}
	else
	{
		EXPECT_FALSE(stateWithNumberOnPosition.isValid());
	}

	OdeState stateWithNumberOnVelocity;
	stateWithNumberOnVelocity.setNumDof(3u, 3u);
	stateWithNumberOnVelocity.getVelocities().setOnes();
	stateWithNumberOnVelocity.getVelocities()[2] = number;
	if (isValid)
	{
		EXPECT_TRUE(stateWithNumberOnVelocity.isValid());
	}
	else
	{
		EXPECT_FALSE(stateWithNumberOnVelocity.isValid());
	}
}
}; // anonymous namespace

TEST(OdeStateTest, IsValidTest)
{
	{
		SCOPED_TRACE("Test with invalid number INF");
		testOdeStateValidityWith(std::numeric_limits<double>::infinity(), false);
	}

	{
		SCOPED_TRACE("Test with invalid number QuietNaN");
		testOdeStateValidityWith(std::numeric_limits<double>::quiet_NaN(), false);
	}

	{
		SCOPED_TRACE("Test with invalid number SignalingNaN");
		testOdeStateValidityWith(std::numeric_limits<double>::signaling_NaN(), false);
	}

	{
		SCOPED_TRACE("Test with valid numbers");
		testOdeStateValidityWith(4.5e-34, true);
	}
}

TEST(OdeStateTest, Interpolation)
{
	OdeState state1;
	OdeState state2;
	OdeState expected;
	state1.setNumDof(3u, 3u);
	state2.setNumDof(3u, 3u);
	expected.setNumDof(3u, 3u);
	double t = 0.25;
	for (size_t i = 0; i < state1.getNumDof(); i++)
	{
		double num = static_cast<double>(i);
		state1.getVelocities()[i] = 1.0;
		state2.getVelocities()[i] = num;
		expected.getVelocities()[i] = 1.0 + (num - 1.0) * t;
		state1.getPositions()[i] = num * 2;
		state2.getPositions()[i] = 0.0;
		expected.getPositions()[i] = num * 2 + (0.0 - num * 2) * t;
	}

	auto result = state1.interpolate(state2, t);
	ASSERT_TRUE(result.isValid());

	for (size_t i = 0; i < expected.getNumDof(); i++)
	{
		EXPECT_NEAR(expected.getPositions()[i], result.getPositions()[i], epsilon);
		EXPECT_NEAR(expected.getVelocities()[i], result.getVelocities()[i], epsilon);
	}

	result = state1.interpolate(state2, 0.0);
	for (size_t i = 0; i < expected.getNumDof(); i++)
	{
		EXPECT_NEAR(state1.getPositions()[i], result.getPositions()[i], epsilon);
		EXPECT_NEAR(state1.getVelocities()[i], result.getVelocities()[i], epsilon);
	}

	result = state1.interpolate(state2, 1.0);
	for (size_t i = 0; i < expected.getNumDof(); i++)
	{
		EXPECT_NEAR(state2.getPositions()[i], result.getPositions()[i], epsilon);
		EXPECT_NEAR(state2.getVelocities()[i], result.getVelocities()[i], epsilon);
	}

}

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

/// \file
/// Tests for the MassSpring3DRepresentation class.

#include <gtest/gtest.h>

#include<SurgSim/Blocks/MassSpring3DRepresentation.h>
using SurgSim::Blocks::MassSpring3DRepresentation;

#include <SurgSim/Physics/DeformableRepresentationState.h>
#include<SurgSim/Physics/LinearSpring.h>
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Physics::LinearSpring;

static void springTest(std::shared_ptr<LinearSpring> spring,
	std::shared_ptr<DeformableRepresentationState> state,
	unsigned int expectedNodeId0, unsigned int expectedNodeId1,
	double expectedStiffness, double expectedDamping)
{
	EXPECT_DOUBLE_EQ(expectedStiffness, spring->getStiffness());
	EXPECT_DOUBLE_EQ(expectedDamping, spring->getDamping());
	EXPECT_EQ(expectedNodeId0, spring->getNodeId(0));
	EXPECT_EQ(expectedNodeId1, spring->getNodeId(1));
	Vector& x = state->getPositions();
	Eigen::VectorBlock<Vector> x0 = SurgSim::Math::getSubVector(x, expectedNodeId0, 3);
	Eigen::VectorBlock<Vector> x1 = SurgSim::Math::getSubVector(x, expectedNodeId1, 3);
	EXPECT_DOUBLE_EQ((x1 - x0).norm(), spring->getRestLength());
}

TEST(MassSpring3DRepresentationTests, init3DTest)
{
	using SurgSim::Math::Vector3d;

	MassSpring3DRepresentation m("MassSpring3D");

	std::array<std::array<std::array<Vector3d, 2>, 2>, 2> extremities =
	{{
		{{
			{{Vector3d::Random(), Vector3d::Random()}},
			{{Vector3d::Random(), Vector3d::Random()}}
		}},
		{{
			{{Vector3d::Random(), Vector3d::Random()}},
			{{Vector3d::Random(), Vector3d::Random()}}
		}}
	}};
	unsigned int numNodesPerDim[3] = {10, 5, 3};
	std::vector<unsigned int> boundaryConditions;
	double totalMass = 1.1;
	double stiffnessStretching = 2.2;
	double dampingStretching = 3.3;
	double stiffnessBending = 4.4;
	double dampingBending = 5.5;
	double stiffnessFaceDiagonal = 6.6;
	double dampingFaceDiagonal = 7.7;
	double stiffnessVolumeDiagonal = 8.8;
	double dampingVolumeDiagonal = 9.9;
	boundaryConditions.push_back(0);
	boundaryConditions.push_back(numNodesPerDim[0] * numNodesPerDim[1] * numNodesPerDim[2] - 1);
	m.init3D(extremities, numNodesPerDim, boundaryConditions,
		totalMass,
		stiffnessStretching, dampingStretching,
		stiffnessBending, dampingBending,
		stiffnessFaceDiagonal, dampingFaceDiagonal,
		stiffnessVolumeDiagonal, dampingVolumeDiagonal);

	const unsigned int numNodes = numNodesPerDim[0] * numNodesPerDim[1] * numNodesPerDim[2];
	EXPECT_EQ(numNodes * 3, m.getNumDof());
	EXPECT_EQ(numNodes, m.getNumMasses());
	unsigned int numSpringsExpected = 0;
	// Stretching springs
	numSpringsExpected += numNodesPerDim[0] * numNodesPerDim[2] * (numNodesPerDim[1] - 1); // along X
	numSpringsExpected += numNodesPerDim[1] * numNodesPerDim[2] * (numNodesPerDim[0] - 1); // along Y
	numSpringsExpected += numNodesPerDim[0] * numNodesPerDim[1] * (numNodesPerDim[2] - 1); // along Z
	// Bending springs
	numSpringsExpected += numNodesPerDim[0] * numNodesPerDim[2] * (numNodesPerDim[1] - 2); // along X
	numSpringsExpected += numNodesPerDim[1] * numNodesPerDim[2] * (numNodesPerDim[0] - 2); // along Y
	numSpringsExpected += numNodesPerDim[0] * numNodesPerDim[1] * (numNodesPerDim[2] - 2); // along Z
	// Face diagonal springs
	numSpringsExpected += numNodesPerDim[1] * (numNodesPerDim[2] - 1) * (numNodesPerDim[0] - 1) * 2; // facing X
	numSpringsExpected += numNodesPerDim[0] * (numNodesPerDim[2] - 1) * (numNodesPerDim[1] - 1) * 2; // facing Y
	numSpringsExpected += numNodesPerDim[2] * (numNodesPerDim[1] - 1) * (numNodesPerDim[0] - 1) * 2; // facing Z
	// Volume diagonal springs
	numSpringsExpected += (numNodesPerDim[0] - 1) * (numNodesPerDim[1] - 1) * (numNodesPerDim[2] - 1) * 4;
	EXPECT_EQ(numSpringsExpected, m.getNumSprings());

	for (unsigned int massId = 0; massId < m.getNumMasses(); massId++)
	{
		EXPECT_DOUBLE_EQ(totalMass/numNodes, m.getMass(massId)->getMass());
	}

	const int depthOffset = numNodesPerDim[0] * numNodesPerDim[1];
	const int rowOffset = numNodesPerDim[0];
	const int colOffset = 1;

	unsigned int springId = 0;
	// The 1st springs are the stretching springs along X
	for (unsigned int depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		for (unsigned int row = 0; row < numNodesPerDim[1]; row++)
		{
			for (unsigned int col = 0; col < numNodesPerDim[0] - 1; col++)
			{
				unsigned int nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + colOffset, stiffnessStretching, dampingStretching);
				springId++;
			}
		}
	}
	// Followed by stretching springs along Y
	for (unsigned int depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		for (unsigned int col = 0; col < numNodesPerDim[0]; col++)
		{
			for (unsigned int row = 0; row < numNodesPerDim[1] - 1; row++)
			{
				unsigned int nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + rowOffset, stiffnessStretching, dampingStretching);
				springId++;
			}
		}
	}
	// Followed by stretching springs along Z
	for (unsigned int row = 0; row < numNodesPerDim[1]; row++)
	{
		for (unsigned int col = 0; col < numNodesPerDim[0]; col++)
		{
			for (unsigned int depth = 0; depth < numNodesPerDim[2] - 1; depth++)
			{
				unsigned int nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + depthOffset, stiffnessStretching, dampingStretching);
				springId++;
			}
		}
	}

	// Followed by bending springs along X
	for (unsigned int depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		for (unsigned int row = 0; row < numNodesPerDim[1]; row++)
		{
			for (unsigned int col = 0; col < numNodesPerDim[0] - 2; col++)
			{
				unsigned int nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + 2 * colOffset, stiffnessBending, dampingBending);
				springId++;
			}
		}
	}
	// Followed by bending springs along Y
	for (unsigned int depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		for (unsigned int col = 0; col < numNodesPerDim[0]; col++)
		{
			for (unsigned int row = 0; row < numNodesPerDim[1] - 2; row++)
			{
				unsigned int nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + 2 * rowOffset, stiffnessBending, dampingBending);
				springId++;
			}
		}
	}
	// Followed by bending springs along Z
	for (unsigned int row = 0; row < numNodesPerDim[1]; row++)
	{
		for (unsigned int col = 0; col < numNodesPerDim[0]; col++)
		{
			for (unsigned int depth = 0; depth < numNodesPerDim[2] - 2; depth++)
			{
				unsigned int nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + 2 * depthOffset, stiffnessBending, dampingBending);
				springId++;
			}
		}
	}

	// Followed by face diagonal springs orthogonal to Z
	for (unsigned int depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		for (unsigned int row = 0; row < numNodesPerDim[1] - 1; row++)
		{
			for (unsigned int col = 0; col < numNodesPerDim[0] - 1; col++)
			{
				unsigned int nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + rowOffset + colOffset, stiffnessFaceDiagonal, dampingFaceDiagonal);
				springId++;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId + colOffset, nodeId + rowOffset, stiffnessFaceDiagonal, dampingFaceDiagonal);
				springId++;
			}
		}
	}
	// Followed by face diagonal springs orthogonal to Y
	for (unsigned int row = 0; row < numNodesPerDim[1]; row++)
	{
		for (unsigned int depth = 0; depth < numNodesPerDim[2] - 1; depth++)
		{
			for (unsigned int col = 0; col < numNodesPerDim[0] - 1; col++)
			{
				unsigned int nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + depthOffset + colOffset, stiffnessFaceDiagonal, dampingFaceDiagonal);
				springId++;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId + colOffset, nodeId + depthOffset, stiffnessFaceDiagonal, dampingFaceDiagonal);
				springId++;
			}
		}
	}
	// Followed by face diagonal springs orthogonal to X
	for (unsigned int col = 0; col < numNodesPerDim[0]; col++)
	{
		for (unsigned int row = 0; row < numNodesPerDim[1] - 1; row++)
		{
			for (unsigned int depth = 0; depth < numNodesPerDim[2] - 1; depth++)
			{
				unsigned int nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + depthOffset + rowOffset, stiffnessFaceDiagonal, dampingFaceDiagonal);
				springId++;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId + rowOffset, nodeId + depthOffset, stiffnessFaceDiagonal, dampingFaceDiagonal);
				springId++;
			}
		}
	}

	// Followed by volume diagonal springs
	for (unsigned int col = 0; col < numNodesPerDim[0] - 1; col++)
	{
		for (unsigned int row = 0; row < numNodesPerDim[1] - 1; row++)
		{
			for (unsigned int depth = 0; depth < numNodesPerDim[2] - 1; depth++)
			{
				unsigned int nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + depthOffset + rowOffset + colOffset,
					stiffnessVolumeDiagonal, dampingVolumeDiagonal);
				springId++;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId + colOffset, nodeId + depthOffset + rowOffset,
					stiffnessVolumeDiagonal, dampingVolumeDiagonal);
				springId++;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId + rowOffset, nodeId + depthOffset + colOffset,
					stiffnessVolumeDiagonal, dampingVolumeDiagonal);
				springId++;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId + rowOffset + colOffset, nodeId + depthOffset,
					stiffnessVolumeDiagonal, dampingVolumeDiagonal);
				springId++;
			}
		}
	}

	// States all equals
	EXPECT_EQ(*m.getPreviousState(), *m.getFinalState());
	EXPECT_EQ(*m.getCurrentState(), *m.getFinalState());
	EXPECT_EQ(*m.getInitialState(), *m.getFinalState());

	// States should contains expected values
	EXPECT_TRUE(m.getFinalState()->getVelocities().isZero());
	EXPECT_TRUE(m.getFinalState()->getAccelerations().isZero());
	EXPECT_FALSE(m.getFinalState()->getPositions().isZero());
	Vector3d depthExtremitiesDelta[2][2] =
	{{(extremities[0][0][1] - extremities[0][0][0]) / static_cast<double>(numNodesPerDim[2] - 1) ,
	(extremities[1][0][1] - extremities[1][0][0]) / static_cast<double>(numNodesPerDim[2] - 1)}
	,
	{(extremities[0][1][1] - extremities[0][1][0]) / static_cast<double>(numNodesPerDim[2] - 1) ,
	(extremities[1][1][1] - extremities[1][1][0]) / static_cast<double>(numNodesPerDim[2] - 1)}};

	unsigned int nodeId = 0;
	for (unsigned int depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		Vector3d depthExtremities[2][2];
		depthExtremities[0][0] = extremities[0][0][0] + depthExtremitiesDelta[0][0] * static_cast<double>(depth);
		depthExtremities[1][0] = extremities[1][0][0] + depthExtremitiesDelta[0][1] * static_cast<double>(depth);
		depthExtremities[0][1] = extremities[0][1][0] + depthExtremitiesDelta[1][0] * static_cast<double>(depth);
		depthExtremities[1][1] = extremities[1][1][0] + depthExtremitiesDelta[1][1] * static_cast<double>(depth);

		Vector3d rowExtremitiesDelta[2] =
		{(depthExtremities[0][1] - depthExtremities[0][0]) / static_cast<double>(numNodesPerDim[1] - 1) ,
		(depthExtremities[1][1] - depthExtremities[1][0]) / static_cast<double>(numNodesPerDim[1] - 1)};
		for (unsigned int row = 0; row < numNodesPerDim[1]; row++)
		{
			Vector3d rowExtremities[2];
			rowExtremities[0] = depthExtremities[0][0] + rowExtremitiesDelta[0] * static_cast<double>(row);
			rowExtremities[1] = depthExtremities[1][0] + rowExtremitiesDelta[1] * static_cast<double>(row);

			Vector3d delta = (rowExtremities[1] - rowExtremities[0]) / static_cast<double>(numNodesPerDim[0] - 1);
			for (unsigned int col = 0; col < numNodesPerDim[0]; col++)
			{
				Vector3d xiExpected(rowExtremities[0] + col * delta);
				Vector& x = m.getFinalState()->getPositions();
				Eigen::VectorBlock<Vector> xi = SurgSim::Math::getSubVector(x, nodeId, 3);
				EXPECT_TRUE(xi.isApprox(xiExpected));
				nodeId++;
			}
		}
	}
	EXPECT_EQ(boundaryConditions, m.getFinalState()->getBoundaryConditions());
}

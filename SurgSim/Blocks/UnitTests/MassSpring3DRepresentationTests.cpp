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

#include "SurgSim/Blocks/MassSpring3DRepresentation.h"
#include "SurgSim/Blocks/UnitTests/SpringTestUtils.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/Mass.h"

using SurgSim::Math::Vector3d;
using SurgSim::Physics::LinearSpring;
using SurgSim::Blocks::MassSpring3DRepresentation;
using SurgSim::Blocks::springTest;

TEST(MassSpring3DRepresentationTests, init3DTest)
{
	MassSpring3DRepresentation m("MassSpring3D");

	std::array<std::array<std::array<Vector3d, 2>, 2>, 2> extremities =
	{{
		{{
			{{Vector3d(-0.5, -0.4, -0.3), Vector3d( 0.4, -0.5, -0.3)}},
			{{Vector3d(-0.4,  0.3, -0.5), Vector3d( 0.5,  0.3, -0.4)}}
		}},
		{{
			{{Vector3d(-0.3, -0.5,  0.4), Vector3d( 0.5, -0.3,  0.4)}},
			{{Vector3d(-0.4,  0.3,  0.5), Vector3d( 0.4,  0.3,  0.5)}}
		}}
	}};
	size_t numNodesPerDim[3] = {10, 5, 3};
	std::vector<size_t> boundaryConditions;
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
	m.initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m.wakeUp();

	const size_t numNodes = numNodesPerDim[0] * numNodesPerDim[1] * numNodesPerDim[2];
	EXPECT_EQ(numNodes * 3, m.getNumDof());
	EXPECT_EQ(numNodes, m.getNumMasses());
	size_t numSpringsExpected = 0;
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

	for (size_t massId = 0; massId < m.getNumMasses(); massId++)
	{
		EXPECT_DOUBLE_EQ(totalMass/numNodes, m.getMass(massId)->getMass());
	}

	const size_t depthOffset = numNodesPerDim[0] * numNodesPerDim[1];
	const size_t rowOffset = numNodesPerDim[0];
	const size_t colOffset = 1;

	size_t springId = 0;
	// The 1st springs are the stretching springs along X
	for (size_t depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		for (size_t row = 0; row < numNodesPerDim[1]; row++)
		{
			for (size_t col = 0; col < numNodesPerDim[0] - 1; col++)
			{
				size_t nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + colOffset, stiffnessStretching, dampingStretching);
				springId++;
			}
		}
	}
	// Followed by stretching springs along Y
	for (size_t depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		for (size_t col = 0; col < numNodesPerDim[0]; col++)
		{
			for (size_t row = 0; row < numNodesPerDim[1] - 1; row++)
			{
				size_t nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + rowOffset, stiffnessStretching, dampingStretching);
				springId++;
			}
		}
	}
	// Followed by stretching springs along Z
	for (size_t row = 0; row < numNodesPerDim[1]; row++)
	{
		for (size_t col = 0; col < numNodesPerDim[0]; col++)
		{
			for (size_t depth = 0; depth < numNodesPerDim[2] - 1; depth++)
			{
				size_t nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + depthOffset, stiffnessStretching, dampingStretching);
				springId++;
			}
		}
	}

	// Followed by bending springs along X
	for (size_t depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		for (size_t row = 0; row < numNodesPerDim[1]; row++)
		{
			for (size_t col = 0; col < numNodesPerDim[0] - 2; col++)
			{
				size_t nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + 2 * colOffset, stiffnessBending, dampingBending);
				springId++;
			}
		}
	}
	// Followed by bending springs along Y
	for (size_t depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		for (size_t col = 0; col < numNodesPerDim[0]; col++)
		{
			for (size_t row = 0; row < numNodesPerDim[1] - 2; row++)
			{
				size_t nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + 2 * rowOffset, stiffnessBending, dampingBending);
				springId++;
			}
		}
	}
	// Followed by bending springs along Z
	for (size_t row = 0; row < numNodesPerDim[1]; row++)
	{
		for (size_t col = 0; col < numNodesPerDim[0]; col++)
		{
			for (size_t depth = 0; depth < numNodesPerDim[2] - 2; depth++)
			{
				size_t nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

				springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
					nodeId, nodeId + 2 * depthOffset, stiffnessBending, dampingBending);
				springId++;
			}
		}
	}

	// Followed by face diagonal springs orthogonal to Z
	for (size_t depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		for (size_t row = 0; row < numNodesPerDim[1] - 1; row++)
		{
			for (size_t col = 0; col < numNodesPerDim[0] - 1; col++)
			{
				size_t nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

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
	for (size_t row = 0; row < numNodesPerDim[1]; row++)
	{
		for (size_t depth = 0; depth < numNodesPerDim[2] - 1; depth++)
		{
			for (size_t col = 0; col < numNodesPerDim[0] - 1; col++)
			{
				size_t nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

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
	for (size_t col = 0; col < numNodesPerDim[0]; col++)
	{
		for (size_t row = 0; row < numNodesPerDim[1] - 1; row++)
		{
			for (size_t depth = 0; depth < numNodesPerDim[2] - 1; depth++)
			{
				size_t nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

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
	for (size_t col = 0; col < numNodesPerDim[0] - 1; col++)
	{
		for (size_t row = 0; row < numNodesPerDim[1] - 1; row++)
		{
			for (size_t depth = 0; depth < numNodesPerDim[2] - 1; depth++)
			{
				size_t nodeId = depth * depthOffset + row * rowOffset + col * colOffset;

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
	EXPECT_FALSE(m.getFinalState()->getPositions().isZero());
	Vector3d depthExtremitiesDelta[2][2] =
	{{(extremities[0][0][1] - extremities[0][0][0]) / static_cast<double>(numNodesPerDim[2] - 1) ,
	(extremities[1][0][1] - extremities[1][0][0]) / static_cast<double>(numNodesPerDim[2] - 1)}
	,
	{(extremities[0][1][1] - extremities[0][1][0]) / static_cast<double>(numNodesPerDim[2] - 1) ,
	(extremities[1][1][1] - extremities[1][1][0]) / static_cast<double>(numNodesPerDim[2] - 1)}};

	size_t nodeId = 0;
	for (size_t depth = 0; depth < numNodesPerDim[2]; depth++)
	{
		Vector3d depthExtremities[2][2];
		depthExtremities[0][0] = extremities[0][0][0] + depthExtremitiesDelta[0][0] * static_cast<double>(depth);
		depthExtremities[1][0] = extremities[1][0][0] + depthExtremitiesDelta[0][1] * static_cast<double>(depth);
		depthExtremities[0][1] = extremities[0][1][0] + depthExtremitiesDelta[1][0] * static_cast<double>(depth);
		depthExtremities[1][1] = extremities[1][1][0] + depthExtremitiesDelta[1][1] * static_cast<double>(depth);

		Vector3d rowExtremitiesDelta[2] =
		{(depthExtremities[0][1] - depthExtremities[0][0]) / static_cast<double>(numNodesPerDim[1] - 1) ,
		(depthExtremities[1][1] - depthExtremities[1][0]) / static_cast<double>(numNodesPerDim[1] - 1)};
		for (size_t row = 0; row < numNodesPerDim[1]; row++)
		{
			Vector3d rowExtremities[2];
			rowExtremities[0] = depthExtremities[0][0] + rowExtremitiesDelta[0] * static_cast<double>(row);
			rowExtremities[1] = depthExtremities[1][0] + rowExtremitiesDelta[1] * static_cast<double>(row);

			Vector3d delta = (rowExtremities[1] - rowExtremities[0]) / static_cast<double>(numNodesPerDim[0] - 1);
			for (size_t col = 0; col < numNodesPerDim[0]; col++)
			{
				Vector3d xiExpected(rowExtremities[0] + static_cast<double>(col) * delta);
				SurgSim::Math::Vector& x = m.getFinalState()->getPositions();
				Eigen::VectorBlock<SurgSim::Math::Vector> xi = SurgSim::Math::getSubVector(x, nodeId, 3);
				EXPECT_TRUE(xi.isApprox(xiExpected));
				nodeId++;
			}
		}
	}

	std::vector<size_t> dofBoundaryConditions;
	for (auto it = boundaryConditions.begin(); it != boundaryConditions.end(); ++it)
	{
		dofBoundaryConditions.push_back((*it) * 3);
		dofBoundaryConditions.push_back((*it) * 3 + 1);
		dofBoundaryConditions.push_back((*it) * 3 + 2);
	}
	EXPECT_EQ(dofBoundaryConditions, m.getFinalState()->getBoundaryConditions());
}

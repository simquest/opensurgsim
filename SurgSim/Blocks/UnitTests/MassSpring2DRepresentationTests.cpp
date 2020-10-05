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
/// Tests for the MassSpring2DRepresentation class.

#include <gtest/gtest.h>

#include "SurgSim/Blocks/MassSpring2DRepresentation.h"
#include "SurgSim/Blocks/UnitTests/SpringTestUtils.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/Mass.h"

using SurgSim::Math::Vector3d;
using SurgSim::Physics::LinearSpring;
using SurgSim::Blocks::MassSpring2DRepresentation;
using SurgSim::Blocks::springTest;

TEST(MassSpring2DRepresentationTests, init2DTest)
{
	MassSpring2DRepresentation m("MassSpring2D");

	std::array<std::array<Vector3d, 2>, 2> extremities =
	{{
		{{Vector3d(0.1, 0.2, 0.3), Vector3d(1.1, 1.2, 1.3)}},
		{{Vector3d(10.1, 10.2, 10.4), Vector3d(9.5, 9.6, 9.3)}}
	}};
	size_t numNodesPerDim[2] = {10, 5};
	std::vector<size_t> boundaryConditions;
	double totalMass = 1.1;
	double stiffnessStretching = 2.2;
	double dampingStretching = 3.3;
	double stiffnessBending = 4.4;
	double dampingBending = 5.5;
	double stiffnessFaceDiagonal = 6.6;
	double dampingFaceDiagonal = 7.7;
	boundaryConditions.push_back(0);
	boundaryConditions.push_back(numNodesPerDim[0] * numNodesPerDim[1] - 1);
	m.init2D(extremities, numNodesPerDim, boundaryConditions,
		totalMass,
		stiffnessStretching, dampingStretching,
		stiffnessBending, dampingBending,
		stiffnessFaceDiagonal, dampingFaceDiagonal);
	m.initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m.wakeUp();


	const size_t numNodes = numNodesPerDim[0] * numNodesPerDim[1];
	EXPECT_EQ(numNodes * 3, m.getNumDof());
	EXPECT_EQ(numNodes, m.getNumMasses());
	size_t numSpringsExpected = 0;
	numSpringsExpected += numNodesPerDim[1] * (numNodesPerDim[0] - 1); // Stretching along Y
	numSpringsExpected += numNodesPerDim[0] * (numNodesPerDim[1] - 1); // Stretching along X
	numSpringsExpected += numNodesPerDim[1] * (numNodesPerDim[0] - 2); // Bending along Y
	numSpringsExpected += numNodesPerDim[0] * (numNodesPerDim[1] - 2); // Bending along X
	numSpringsExpected += (numNodesPerDim[0] - 1) * (numNodesPerDim[1] - 1) * 2; // Face diagonal
	EXPECT_EQ(numSpringsExpected, m.getNumSprings());

	for (size_t massId = 0; massId < m.getNumMasses(); massId++)
	{
		EXPECT_DOUBLE_EQ(totalMass/numNodes, m.getMass(massId)->getMass());
	}

	const size_t rowOffset = numNodesPerDim[0];
	const size_t colOffset = 1;

	size_t springId = 0;
	// The 1st springs are the stretching springs along X
	for (size_t row = 0; row < numNodesPerDim[1]; row++)
	{
		for (size_t col = 0; col < numNodesPerDim[0] - 1; col++)
		{
			size_t nodeId = row * rowOffset + col * colOffset;

			springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
				nodeId, nodeId + colOffset, stiffnessStretching, dampingStretching);
			springId++;
		}
	}
	// Followed by stretching springs along Y
	for (size_t col = 0; col < numNodesPerDim[0]; col++)
	{
		for (size_t row = 0; row < numNodesPerDim[1] - 1; row++)
		{
			size_t nodeId = row * rowOffset + col * colOffset;

			springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
				nodeId, nodeId + rowOffset, stiffnessStretching, dampingStretching);
			springId++;
		}
	}
	// Followed by bending springs along X
	for (size_t row = 0; row < numNodesPerDim[1]; row++)
	{
		for (size_t col = 0; col < numNodesPerDim[0] - 2; col++)
		{
			size_t nodeId = row * rowOffset + col * colOffset;

			springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
				nodeId, nodeId + 2 * colOffset, stiffnessBending, dampingBending);
			springId++;
		}
	}
	// Followed by bending springs along Y
	for (size_t col = 0; col < numNodesPerDim[0]; col++)
	{
		for (size_t row = 0; row < numNodesPerDim[1] - 2; row++)
		{
			size_t nodeId = row * rowOffset + col * colOffset;

			springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
				nodeId, nodeId + 2 * rowOffset, stiffnessBending, dampingBending);
			springId++;
		}
	}
	// Followed by face diagonal springs
	for (size_t row = 0; row < numNodesPerDim[1] - 1; row++)
	{
		for (size_t col = 0; col < numNodesPerDim[0] - 1; col++)
		{
			size_t nodeId = row * rowOffset + col * colOffset;

			springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
				nodeId, nodeId + rowOffset + colOffset, stiffnessFaceDiagonal, dampingFaceDiagonal);
			springId++;

			springTest( std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
				nodeId + colOffset, nodeId + rowOffset, stiffnessFaceDiagonal, dampingFaceDiagonal);
			springId++;
		}
	}

	// States all equals
	EXPECT_EQ(*m.getPreviousState(), *m.getFinalState());
	EXPECT_EQ(*m.getCurrentState(), *m.getFinalState());
	EXPECT_EQ(*m.getInitialState(), *m.getFinalState());

	// States should contains expected values
	EXPECT_TRUE(m.getFinalState()->getVelocities().isZero());
	EXPECT_FALSE(m.getFinalState()->getPositions().isZero());
	Vector3d rowExtremititiesDelta[2] =
	{
		(extremities[0][1] - extremities[0][0]) / static_cast<double>(numNodesPerDim[1] - 1) ,
		(extremities[1][1] - extremities[1][0]) / static_cast<double>(numNodesPerDim[1] - 1)
	};
	size_t nodeId = 0;
	for (size_t row = 0; row < numNodesPerDim[1]; row++)
	{
		Vector3d rowExtremities[2];
		rowExtremities[0] = extremities[0][0] + rowExtremititiesDelta[0] * static_cast<double>(row);
		rowExtremities[1] = extremities[1][0] + rowExtremititiesDelta[1] * static_cast<double>(row);

		Vector3d delta = (rowExtremities[1] - rowExtremities[0]) / static_cast<double>(numNodesPerDim[0] - 1);
		for (size_t col = 0; col < numNodesPerDim[0]; col++)
		{
			SurgSim::Math::Vector3d piExpected(rowExtremities[0] + static_cast<double>(col) * delta);
			SurgSim::Math::Vector& x = m.getFinalState()->getPositions();
			Eigen::VectorBlock<SurgSim::Math::Vector> pi = SurgSim::Math::getSubVector(x, nodeId, 3);
			EXPECT_TRUE(pi.isApprox(piExpected));
			nodeId++;
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

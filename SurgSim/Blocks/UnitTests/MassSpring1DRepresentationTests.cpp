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
/// Tests for the MassSpring1DRepresentation class.

#include <gtest/gtest.h>

#include <SurgSim/Blocks/MassSpring1DRepresentation.h>
#include <SurgSim/Blocks/UnitTests/SpringTestUtils.h>
#include <SurgSim/Physics/LinearSpring.h>

using SurgSim::Math::Vector3d;
using SurgSim::Physics::LinearSpring;
using SurgSim::Blocks::MassSpring1DRepresentation;
using SurgSim::Blocks::springTest;

TEST(MassSpring1DRepresentationTests, init1DTest)
{
	MassSpring1DRepresentation m("MassSpring1D");

	std::array<Vector3d, 2> extremities = {{ Vector3d(1.1, 1.2, 1.3), Vector3d(2.2, 2.3, 2.4) }};
	unsigned int numNodesPerDim[1] = {10};
	std::vector<unsigned int> boundaryConditions;
	double totalMass = 1.1;
	double stiffnessStretching = 2.2;
	double dampingStretching = 3.3;
	double stiffnessBending = 4.4;
	double dampingBending = 5.5;
	boundaryConditions.push_back(0);
	boundaryConditions.push_back(numNodesPerDim[0] - 1);
	m.init1D(extremities, numNodesPerDim, boundaryConditions,
		totalMass, stiffnessStretching, dampingStretching, stiffnessBending, dampingBending);

	EXPECT_EQ(numNodesPerDim[0] * 3, m.getNumDof());
	EXPECT_EQ(numNodesPerDim[0], m.getNumMasses());
	EXPECT_EQ(numNodesPerDim[0] - 1 + numNodesPerDim[0] - 2, m.getNumSprings());

	for (unsigned int massId = 0; massId < m.getNumMasses(); massId++)
	{
		EXPECT_DOUBLE_EQ(totalMass/numNodesPerDim[0], m.getMass(massId)->getMass());
	}

	// The 1st springs are the stretching springs
	unsigned int springId = 0;
	for (unsigned int nodeId = 0; nodeId < m.getNumMasses() - 1; nodeId++)
	{
		springTest(std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
			nodeId, nodeId + 1, stiffnessStretching, dampingStretching);
		springId++;
	}

	// Followed by bending springs
	for (unsigned int nodeId = 0; nodeId < m.getNumMasses() - 2; nodeId++)
	{
		springTest(std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
			nodeId, nodeId + 2, stiffnessBending, dampingBending);
		springId++;
	}

	// States all equals
	EXPECT_EQ(*m.getPreviousState(), *m.getFinalState());
	EXPECT_EQ(*m.getCurrentState(), *m.getFinalState());
	EXPECT_EQ(*m.getInitialState(), *m.getFinalState());

	// States should contains expected values
	Vector3d delta = (extremities[1] - extremities[0]) / (numNodesPerDim[0] - 1);
	EXPECT_TRUE(m.getFinalState()->getVelocities().isZero());
	EXPECT_TRUE(m.getFinalState()->getAccelerations().isZero());
	EXPECT_FALSE(m.getFinalState()->getPositions().isZero());
	for (unsigned int nodeId = 0; nodeId < numNodesPerDim[0]; nodeId++)
	{
		Vector3d piExpected = extremities[0] + delta * nodeId;
		Eigen::VectorBlock<Vector> pi = SurgSim::Math::getSubVector(m.getFinalState()->getPositions(), nodeId, 3);
		EXPECT_TRUE(pi.isApprox(piExpected));
	}
	EXPECT_EQ(boundaryConditions, m.getFinalState()->getBoundaryConditions());
}

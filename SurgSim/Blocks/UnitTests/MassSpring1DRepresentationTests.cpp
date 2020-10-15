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

#include "SurgSim/Blocks/MassSpring1DRepresentation.h"
#include "SurgSim/Blocks/UnitTests/SpringTestUtils.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/Mass.h"

using SurgSim::Math::Vector3d;
using SurgSim::Physics::LinearSpring;
using SurgSim::Blocks::MassSpring1DRepresentation;
using SurgSim::Blocks::springTest;

TEST(MassSpring1DRepresentationTests, init1DTest)
{
	MassSpring1DRepresentation m("MassSpring1D");

	std::array<Vector3d, 2> extremities = {{ Vector3d(1.1, 1.2, 1.3), Vector3d(2.2, 2.3, 2.4) }};
	size_t numNodesPerDim[1] = {10};

	std::vector<Vector3d> nodes;
	for (size_t nodeId = 0; nodeId < numNodesPerDim[0]; ++nodeId)
	{
		double abscissa = static_cast<double>(nodeId) / static_cast<double>(numNodesPerDim[0] - 1);
		nodes.push_back(extremities[0] + abscissa * (extremities[1] - extremities[0]));
	}

	std::vector<size_t> boundaryConditions;
	double totalMass = 1.1;
	double stiffnessStretching = 2.2;
	double dampingStretching = 3.3;
	double stiffnessBending = 4.4;
	double dampingBending = 5.5;
	boundaryConditions.push_back(0);
	boundaryConditions.push_back(numNodesPerDim[0] - 1);
	m.init1D(nodes, boundaryConditions,
		totalMass, stiffnessStretching, dampingStretching, stiffnessBending, dampingBending);
	m.initialize(std::make_shared<SurgSim::Framework::Runtime>());
	m.wakeUp();

	EXPECT_EQ(numNodesPerDim[0] * 3, m.getNumDof());
	EXPECT_EQ(numNodesPerDim[0], m.getNumMasses());
	EXPECT_EQ(numNodesPerDim[0] - 1 + numNodesPerDim[0] - 2, m.getNumSprings());

	for (size_t massId = 0; massId < m.getNumMasses(); massId++)
	{
		EXPECT_DOUBLE_EQ(totalMass/numNodesPerDim[0], m.getMass(massId)->getMass());
	}

	// The 1st springs are the stretching springs
	size_t springId = 0;
	for (size_t nodeId = 0; nodeId < m.getNumMasses() - 1; nodeId++)
	{
		springTest(std::dynamic_pointer_cast<LinearSpring>(m.getSpring(springId)), m.getFinalState(),
			nodeId, nodeId + 1, stiffnessStretching, dampingStretching);
		springId++;
	}

	// Followed by bending springs
	for (size_t nodeId = 0; nodeId < m.getNumMasses() - 2; nodeId++)
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
	Vector3d delta = (extremities[1] - extremities[0]) / static_cast<double>(numNodesPerDim[0] - 1);
	EXPECT_TRUE(m.getFinalState()->getVelocities().isZero());
	EXPECT_FALSE(m.getFinalState()->getPositions().isZero());
	for (size_t nodeId = 0; nodeId < numNodesPerDim[0]; nodeId++)
	{
		Vector3d piExpected = extremities[0] + delta * static_cast<double>(nodeId);
		SurgSim::Math::Vector& x = m.getFinalState()->getPositions();
		Eigen::VectorBlock<SurgSim::Math::Vector> pi = SurgSim::Math::getSubVector(x, nodeId, 3);
		EXPECT_TRUE(pi.isApprox(piExpected));
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

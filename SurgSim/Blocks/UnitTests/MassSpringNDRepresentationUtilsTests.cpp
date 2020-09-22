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

/// \file
/// Tests for the MassSpring1DRepresentation class.

#include <memory>
#include <gtest/gtest.h>

#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/LinearSpring.h"

using SurgSim::Physics::LinearSpring;

TEST(MassSpringNDRepresentationUtilsTests, CreateLinearSpring)
{
	const int numDofPerNode = 3;
	const int numNodes = 10;
	double stiffness = 14.54;
	double damping = 4.398;

	{
		SCOPED_TRACE("Create linear spring with 0 rest-length");
		std::shared_ptr<SurgSim::Math::OdeState> state;
		state = std::make_shared<SurgSim::Math::OdeState>();
		state->setNumDof(numDofPerNode, numNodes);

		std::shared_ptr<LinearSpring> linearSpring;
		linearSpring = SurgSim::Physics::createLinearSpring(state, 3, 7, stiffness, damping);
		EXPECT_DOUBLE_EQ(damping, linearSpring->getDamping());
		EXPECT_DOUBLE_EQ(stiffness, linearSpring->getStiffness());
		EXPECT_DOUBLE_EQ(0.0, linearSpring->getRestLength());
		EXPECT_EQ(2u, linearSpring->getNumNodes());
		EXPECT_EQ(3u, linearSpring->getNodeId(0));
		EXPECT_EQ(7u, linearSpring->getNodeId(1));
		EXPECT_EQ(2u, linearSpring->getNodeIds().size());
		EXPECT_EQ(3u, linearSpring->getNodeIds()[0]);
		EXPECT_EQ(7u, linearSpring->getNodeIds()[1]);
	}

	{
		SCOPED_TRACE("Create linear spring with non 0 rest-length");
		std::shared_ptr<SurgSim::Math::OdeState> state;
		state = std::make_shared<SurgSim::Math::OdeState>();
		state->setNumDof(numDofPerNode, numNodes);
		for (int nodeId = 0; nodeId < numNodes; nodeId++)
		{
			SurgSim::Math::Vector& x = state->getPositions();
			SurgSim::Math::getSubVector(x, nodeId, numDofPerNode)[0] = static_cast<double>(nodeId) + 0.45009;
			SurgSim::Math::getSubVector(x, nodeId, numDofPerNode)[1] = -4.5343;
			SurgSim::Math::getSubVector(x, nodeId, numDofPerNode)[2] = 0.2325445;
		}

		std::shared_ptr<LinearSpring> linearSpring;
		linearSpring = SurgSim::Physics::createLinearSpring(state, 3, 7, stiffness, damping);
		EXPECT_DOUBLE_EQ(damping, linearSpring->getDamping());
		EXPECT_DOUBLE_EQ(stiffness, linearSpring->getStiffness());
		EXPECT_DOUBLE_EQ(4.0, linearSpring->getRestLength());
		EXPECT_EQ(2u, linearSpring->getNumNodes());
		EXPECT_EQ(3u, linearSpring->getNodeId(0));
		EXPECT_EQ(7u, linearSpring->getNodeId(1));
		EXPECT_EQ(2u, linearSpring->getNodeIds().size());
		EXPECT_EQ(3u, linearSpring->getNodeIds()[0]);
		EXPECT_EQ(7u, linearSpring->getNodeIds()[1]);
	}
}

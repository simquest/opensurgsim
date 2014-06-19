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
/// Utility function to test a LinearSpring

#include <gtest/gtest.h>

#include "SurgSim/Blocks/UnitTests/SpringTestUtils.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/LinearSpring.h"

namespace SurgSim
{

namespace Blocks
{

void springTest(std::shared_ptr<SurgSim::Physics::LinearSpring> spring,
	std::shared_ptr<SurgSim::Math::OdeState> state,
	size_t expectedNodeId0, size_t expectedNodeId1,
	double expectedStiffness, double expectedDamping)
{
	EXPECT_DOUBLE_EQ(expectedStiffness, spring->getStiffness());
	EXPECT_DOUBLE_EQ(expectedDamping, spring->getDamping());
	EXPECT_EQ(expectedNodeId0, spring->getNodeId(0));
	EXPECT_EQ(expectedNodeId1, spring->getNodeId(1));
	SurgSim::Math::Vector& x =state->getPositions();
	Eigen::VectorBlock<SurgSim::Math::Vector> x0 = SurgSim::Math::getSubVector(x, expectedNodeId0, 3);
	Eigen::VectorBlock<SurgSim::Math::Vector> x1 = SurgSim::Math::getSubVector(x, expectedNodeId1, 3);
	EXPECT_DOUBLE_EQ((x1 - x0).norm(), spring->getRestLength());
}

}; // namespace Blocks

}; // namespace SurgSim

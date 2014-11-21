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

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/ParticlesState.h"


namespace SurgSim
{
namespace Particles
{

TEST(ParticlesStateTests, InitTest)
{
	ASSERT_NO_THROW(ParticlesState state;);
}

TEST(ParticlesStateTests, AllocateTest)
{
	ParticlesState state;
	EXPECT_EQ(0u, state.getNumDof());
	EXPECT_EQ(0u, state.getNumNodes());
	EXPECT_EQ(0u, state.getNumBoundaryConditions());
	EXPECT_EQ(0, state.getBoundaryConditions().size());
	EXPECT_EQ(0, state.getPositions().size());
	EXPECT_EQ(0, state.getVelocities().size());
	EXPECT_EQ(0, state.getLifetimes().size());

	ASSERT_NO_THROW(state.setNumDof(3u, 4u));
	EXPECT_EQ(12u, state.getNumDof());
	EXPECT_EQ(4u, state.getNumNodes());
	EXPECT_EQ(12, state.getPositions().size());
	EXPECT_EQ(12, state.getVelocities().size());
	EXPECT_EQ(4, state.getLifetimes().size());
	EXPECT_EQ(0u , state.getNumBoundaryConditions());
	EXPECT_EQ(0 , state.getBoundaryConditions().size());
}

TEST(ParticlesStateTests, LifetimesTest)
{
	ParticlesState state;
	state.setNumDof(3u, 5u);
	EXPECT_TRUE(state.getLifetimes().isZero());

	state.getLifetimes() << 0, 1, 2, 3, 4;
	for (int i = 0; i < 5; i++)
	{
		EXPECT_NEAR(i, state.getLifetimes()[i], 1e-9);
	}
}


TEST(ParticlesStateTests, ResetTest)
{
	ParticlesState state;
	state.setNumDof(3u, 5u);
	state.getLifetimes() << 0, 1, 2, 3, 4;
	for (int i = 0; i < 5; i++)
	{
		EXPECT_NEAR(i, state.getLifetimes()[i], 1e-9);
	}

	ParticlesState zeroState;
	zeroState.setNumDof(3u, 5u);
	EXPECT_NE(zeroState, state);

	state.reset();
	EXPECT_EQ(zeroState, state);
}

TEST(ParticlesStateTests, IsValidTest)
{
	ParticlesState state;
	state.setNumDof(3u, 3u);
	state.getLifetimes().setOnes();
	EXPECT_TRUE(state.isValid());

	state.getLifetimes()[2] = std::numeric_limits<double>::infinity();
	EXPECT_FALSE(state.isValid());
}

}; // namespace Particles
}; // namespace SurgSim
